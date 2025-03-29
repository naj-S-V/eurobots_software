#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utils.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

// ================================================================
//                           Connection
// ================================================================

#define CPT_LEFT 3
#define CPT_RIGHT 6
#define CPT_VOID 7
#define CPT_US_RIGHT_TRIG_PIN 10
#define CPT_US_RIGHT_ECHO_PIN 8
#define CPT_US_LEFT_TRIG_PIN 13
#define CPT_US_LEFT_ECHO_PIN 11
#define RELAY 2

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 65;

const int stopAfterSec = 15;
const int delayStartSec = 5;
const float obstacleThreshold = 10.0;
const int ultrasonicInterval = 50;

// ================================================================
//                           Initialisation
// ================================================================

int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);
float obstacleRightDistance = 0.0;
float obstacleLeftDistance = 0.0;
unsigned long startTime;
unsigned long lastUltrasonicCheck = 0;
bool currentSensor = false;

// ================================================================
//                           FSM States
// ================================================================
enum State {
  IDLE,
  START,
  RUNNING,
  AVOID_OBSTACLE,
  STOPPED
};

State currentState = IDLE;

// ================================================================
//                           Setup
// ================================================================

void setup() {

  Serial.begin(9600);

  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);
  pinMode(CPT_VOID, INPUT);

  pinMode(CPT_US_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_RIGHT_ECHO_PIN, INPUT);
  pinMode(CPT_US_LEFT_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_LEFT_ECHO_PIN, INPUT);

  pinMode(RELAY, INPUT_PULLUP);
  
  AFMS.begin();
}

// ================================================================
//                           Loop (FSM Logic)
// ================================================================
void loop() {
  unsigned long elapsedTime = millis() - startTime;

  test();
  
  updateUltrasonicReadings();
  
  switch (currentState) {
    
    case IDLE:
      if(digitalRead(RELAY) == HIGH){
        startTime = millis();
        currentState = START;
      }
      break;
      
    case START:
      if (elapsedTime >= delayStartSec * 1000) {
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      if (digitalRead(CPT_VOID) ||
         (obstacleRightDistance <= obstacleThreshold && obstacleRightDistance != 0) ||
         (obstacleLeftDistance <= obstacleThreshold && obstacleLeftDistance != 0))
        {
        currentState = AVOID_OBSTACLE;
      } else if (elapsedTime >= maxTime) {
        currentState = STOPPED;
      } else {
        moveRobot();
      }
      break;
    
    case AVOID_OBSTACLE:
      stopMotors();
      currentState = RUNNING;
      break;
    
    case STOPPED:
      stopMotors();
      break;
  }
}

// ================================================================
//                           Functions
// ================================================================
void moveRobot() {
  bool left = digitalRead(CPT_LEFT);   
  bool right = digitalRead(CPT_RIGHT);

  if (!left && right) {
    motorLeft->run(RELEASE);
    motorRight->setSpeed(speed);
    motorRight->run(FORWARD);
  } else if (left && !right) {
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
  } else {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
}

/*
  stopMotors()
  Description: Stop the motors.
  Parameters: None
  Returns: None
*/
void stopMotors() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

/*
  readUltrasonic(int trigPin, int echoPin)
  Description: Measures the distance using an ultrasonic sensor.
  Parameters: 
    - trigPin (int): The trigger pin of the ultrasonic sensor.
    - echoPin (int): The echo pin of the ultrasonic sensor.
  Returns: 
    - float: The measured distance in centimeters.
*/
float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 10000);
  return (duration * .0343) / 2; 
}

/*
  updateUltrasonicReadings()
  Description: Updates the ultrasonic sensor readings at a fixed interval.
  Parameters: None
  Returns: None
*/
void updateUltrasonicReadings() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastUltrasonicCheck >= ultrasonicInterval) {
    lastUltrasonicCheck = currentTime;
    
    if (currentSensor) {
      obstacleLeftDistance = readUltrasonic(CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN);
    } else {
      obstacleRightDistance = readUltrasonic(CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN);
    }
    
    currentSensor = !currentSensor;
  }
}



