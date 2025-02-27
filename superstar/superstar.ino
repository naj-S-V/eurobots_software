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

// #define CPT_US_RIGHT_TRIG_PIN 8
// #define CPT_US_RIGHT_ECHO_PIN 11

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 65;

const float threshold = 10.0;

const bool motOff = false;

const int stopAfterSec = 10;

const int delayStartSec = 5; 

// ================================================================
//                           Initilialisation
// ================================================================

int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);
float distance = 0.0;


// ================================================================
//                           Setup
// ================================================================
void setup() {
  Serial.begin(9600);
  delay(delayStartSec * 1000);
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);
  pinMode(CPT_VOID, INPUT);

  // pinMode(CPT_US_RIGHT_TRIG_PIN, OUTPUT);
  // pinMode(CPT_US_RIGHT_ECHO_PIN, INPUT);

  AFMS.begin();
}

// ================================================================
//                           Loop
// ================================================================
void loop() {
  Serial.println(String(maxTime) + "-" + String(millis()));

  bool isVoid = digitalRead(CPT_VOID);

  // digitalWrite(CPT_US_RIGHT_TRIG_PIN, LOW);
  // delayMicroseconds(2);
  // digitalWrite(CPT_US_RIGHT_TRIG_PIN, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(CPT_US_RIGHT_TRIG_PIN, LOW);

  // long duration = pulseIn(CPT_US_RIGHT_ECHO_PIN, HIGH, 30000);
  // distance = (duration * .0343) / 2;

  // if (distance <= threshold && distance != 0) {
  //   motorLeft->run(RELEASE);
  //   motorRight->run(RELEASE);
  //   Serial.println(String(distance) + ": halte");
  //   delay(10);
  //   return;     
  // } else {
  //   Serial.println(distance);
  // }

  if (isVoid) {
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    // Serial.println("halte");
    delay(10);
    return;     
  }

  if(motOff || maxTime <= millis()){
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    Serial.println("Halte motor !");
    delay(10);
    return;
  }

  bool left = digitalRead(CPT_LEFT);   
  bool right = digitalRead(CPT_RIGHT);

  if (!left && right) {
    motorLeft->run(RELEASE);
    motorRight->setSpeed(speed);
    motorRight->run(FORWARD);
    Serial.println("Left");
  } else if (left && !right) {
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
    Serial.println("Right");
  } else {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    Serial.println("Forward");
  }
  delay(10);
}