#include <Wire.h>
#include <Adafruit_MotorShield.h>

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

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 65;

const bool motOff = false;
const int stopAfterSec = 10000;
const int delayStartSec = 5;
const float obstacleThreshold = 20.0;
const int ultrasonicInterval = 100;

// ================================================================
//                           Initialisation
// ================================================================

int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);
float obstacleDistance = 0.0;
unsigned long startTime;

unsigned long lastUltrasonicCheck = 0;

// ================================================================
//                           FSM States
// ================================================================
enum State {
  IDLE,
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
  
  AFMS.begin();
  startTime = millis();
}

// ================================================================
//                           Loop (FSM Logic)
// ================================================================
void loop() {
  unsigned long elapsedTime = millis() - startTime;

  unsigned long currentTime = millis();

  if (currentTime - lastUltrasonicCheck >= ultrasonicInterval) {
    lastUltrasonicCheck = currentTime;
    obstacleDistance = readUS();
  }
  
  switch (currentState) {
    
    case IDLE:
      Serial.println("Waiting to start...");
      if (elapsedTime >= delayStartSec * 1000) {
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      Serial.println("Moving...");
      if (digitalRead(CPT_VOID)) {
        currentState = AVOID_OBSTACLE;
        break;
      } else if (obstacleDistance <= obstacleThreshold && obstacleDistance != 0){
        currentState = AVOID_OBSTACLE;
        break;
      } else if (motOff || elapsedTime >= maxTime) {
        currentState = STOPPED;
        break;
      }
      moveRobot();
      break;
    
    case AVOID_OBSTACLE:
      Serial.println("Obstacle detected! Stopping...");
      stopMotors();
      currentState = RUNNING;
      break;
    
    case STOPPED:
      Serial.println("Halte motor !");
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
    Serial.println("Turning Left");
  } else if (left && !right) {
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
    Serial.println("Turning Right");
  } else {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    Serial.println("Moving Forward");
  }
}

void stopMotors() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

float readUS(){
  digitalWrite(CPT_US_RIGHT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_RIGHT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_RIGHT_TRIG_PIN, LOW);

  long duration = pulseIn(CPT_US_RIGHT_ECHO_PIN, HIGH, 4000);
  return (duration * .0343) / 2; 
}

