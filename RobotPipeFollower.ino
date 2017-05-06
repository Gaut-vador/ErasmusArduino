//ZumoMotor library
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>

//Pixy camera library
#include <SPI.h>  
#include <Pixy.h>


#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)

//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop{
public:
  ServoLoop(int32_t proportionalGain, int32_t derivativeGain);

  void update(int32_t error);

  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_proportionalGain;
  int32_t m_derivativeGain;
};

// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain){
  m_pos = RCS_CENTER_POS;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;
}

// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error){
  long int velocity;
  //char buf[32];
  if (m_prevError!=0x80000000){	
    velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;

    m_pos += velocity;
    if (m_pos>RCS_MAX_POS){
      m_pos = RCS_MAX_POS; 
    }
    else if (m_pos<RCS_MIN_POS){
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------

Pixy pixy;  // Declare the camera object

ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt




















ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
int lastError = 0;

//Max speed of the Zumo motor, degrade this value and the robot will go slower (Max 400)
const int MAX_SPEED = 400;

//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup(){
  //Little song at the begining
  buzzer.play(">g32>>c32");

  // Initialise the infrared sensor
  reflectanceSensors.init();
  //Wait for the button of the zumo to be pressed
  button.waitForButton();

  //Light up the LED to show we are in configuration mode
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  //Initialization of the Pixy camera
  pixy.init();

  // Wait 1 seconde and start the calibration of the sensor by rotate the robot to put the sensor on the line
  delay(1000);
  int i;
  for(i = 0; i < 80; i++){
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
    
    reflectanceSensors.calibrate();

    // Total time of the calibration is
    // 80*20 = 1600 ms.
    delay(20);
  }
  motors.setSpeeds(0,0);

  // Shut down the LED and play sound to show we finnished the calibration
  digitalWrite(13, LOW);
  buzzer.play(">g32>>c32");
}

//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop(){
  moveRobot();
}


//---------------------------------------
// Allow to move the robot by following the line
//---------------------------------------
void moveRobot(){
  
  unsigned int sensors[6];

  // Obtenir la position de la ligne.  Notez qu'il FAUT fournit le senseur "sensors"
  // en argument à la fonction readLine(), même si nous ne sommes intéressé
  // par les lectures individuelles des différents senseurs.
  int position = reflectanceSensors.readLine(sensors);

  // L'erreur ("error") est la distance par rapport au centre de la ligne, qui 
  // correspond à la position 2500.
  int error = position - 2500;

  // Calculer la différence de vitesse (speedDifference) entre les moteurs 
  // en utilisant un les termes proportionnels et dérivés du régulateur PID.
  // (Le terme intégral n'est généralement pas très utile dans le 
  // suivit de ligne).
  // Nous utiliserons 1/4 pour la constante proportionnelle et 6 pour la 
  // constante dérivée 6, qui devrait fonctionner correctement avec de 
  // nombreux choix de Zumo.
  // Vous aurez probablement besoin d'ajuster ces constantes par
  // essai/erreur pour votre zumo et/ou le circuit.
  int speedDifference = error / 4 + 6 * (error - lastError);

  lastError = error;

  // Calculer la vitesse de chaque moteur. Le signe de la différence (speedDifference)
  // determine si le moteur tourne à gauche ou a droite.
  int m1Speed = MAX_SPEED + speedDifference;
  int m2Speed = MAX_SPEED - speedDifference;

  // Nous allons contraindre la vitesse des moteurs entre 0 et MAX_SPEED.
  // D'une façon générale, un des moteurs est toujours à MAX_SPEED
  // et l'autre sera à MAX_SPEED-|speedDifference| si elle est positif,
  // sinon il sera en vitesse stationnaire. Pour certaines applications, 
  // vous pourriez désirer une vitesse négative, ce qui permettrai de
  // tourner à l'envers.
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  if (m1Speed > MAX_SPEED)
    m1Speed = MAX_SPEED;
  if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;

  motors.setSpeeds(m1Speed, m2Speed);
}
