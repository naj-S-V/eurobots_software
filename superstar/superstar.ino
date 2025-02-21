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

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 65;

const bool motOff = false;


// ================================================================
//                           Setup
// ================================================================
void setup() {
  Serial.begin(9600);
  delay(5000);
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);
  pinMode(CPT_VOID, INPUT);

  AFMS.begin();
}

// ================================================================
//                           Loop
// ================================================================
void loop() {
  bool isVoid = digitalRead(CPT_VOID);

  if (isVoid) {
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    Serial.println("halte");
    delay(10);
    return;     
  }

  if(motOff){
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