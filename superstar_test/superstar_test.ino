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
#define CPT_US_LEFT_TRIG_PIN 13
#define CPT_US_LEFT_ECHO_PIN 11

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 65;

const bool motOff = false;
const int stopAfterSec = 15;
const int delayStartSec = 5;
const float obstacleThreshold = 25.0;
const int ultrasonicInterval = 200;

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
    
    if (currentSensor) {
      obstacleLeftDistance = readUltrasonic(CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN);
    } else {
      obstacleRightDistance = readUltrasonic(CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN);
    }
    
    currentSensor = !currentSensor;
  }
  
  switch (currentState) {
    
    case IDLE:
      // Serial.println("Waiting to start...");
      if (elapsedTime >= delayStartSec * 1000) {
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      // Serial.println("Moving...");
      if (digitalRead(CPT_VOID)) {
        currentState = AVOID_OBSTACLE;
      } else if (obstacleRightDistance <= obstacleThreshold && obstacleRightDistance != 0){
        currentState = AVOID_OBSTACLE;
      } else if (obstacleLeftDistance <= obstacleThreshold && obstacleLeftDistance != 0){
        currentState = AVOID_OBSTACLE;
      } else if (motOff || elapsedTime >= maxTime) {
        currentState = STOPPED;
      } else {
        moveRobot();
      }
      break;
    
    case AVOID_OBSTACLE:
      // Serial.println("Obstacle detected! Stopping...");
      stopMotors();
      currentState = RUNNING;
      break;
    
    case STOPPED:
      // Serial.println("Halte motor !");
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
    // Serial.println("Turning Left");
  } else if (left && !right) {
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
    // Serial.println("Turning Right");
  } else {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    // Serial.println("Moving Forward");
  }
}

void stopMotors() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 4000);
  return (duration * .0343) / 2; 
}

