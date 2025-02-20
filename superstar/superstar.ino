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
#define CPT_US_TRIG_PIN 13
#define CPT_US_ECHO_PIN 8

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 65;

float distance = 0.0;
const float threshold = 5.0;

volatile int counter = 1;
const int maxInt = 0;


// ================================================================
//                           Setup
// ================================================================
void setup() {
  Serial.begin(9600);
  delay(5000);
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);

  pinMode(CPT_US_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_ECHO_PIN, INPUT);

  AFMS.begin();
}

// ================================================================
//                           Loop
// ================================================================
void loop() {
  digitalWrite(CPT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_TRIG_PIN, LOW);

  long duration = pulseIn(CPT_US_ECHO_PIN, HIGH, 10000);
  distance = (duration * .0343) / 2; 

  // Check if this is a good solution.
  // counter = counter + 1;
  // if (counter >= 600){
  //   speed = 40;
  // }

  if (distance >= threshold) {
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    Serial.println(String(distance) + ": halte");
    delay(10);
    return;     
  } else {
    Serial.println(distance);
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