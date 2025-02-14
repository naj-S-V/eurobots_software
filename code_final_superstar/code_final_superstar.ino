#include <Wire.h>
#include <Adafruit_MotorShield.h>


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

// #define CPT_LEFT 7       // Broche pour le capteur gauche
// #define CPT_RIGHT 8     // Broche pour le capteur droit

// #define CPT_US_TRIG_PIN 12 // Broche TRIG pour le capteur ultrason
// #define CPT_US_ECHO_PIN 13 // Broche ECHO pour le capteur ultrason

#define CPT_LEFT 7
#define CPT_RIGHT 8
#define CPT_US_TRIG_PIN 12
#define CPT_US_ECHO_PIN 9

volatile int speed = 65;

float distance = 0.0;
const float seuil = 5.0;
bool isSurfaceBelow = false;

volatile int counter = 1;
const int max_int = 0;



void setup() {
  delay(5000);
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);

  pinMode(CPT_US_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_ECHO_PIN, INPUT);

  AFMS.begin();

  
}

void loop() {
  
  digitalWrite(CPT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_TRIG_PIN, LOW);

  long duration = pulseIn(CPT_US_ECHO_PIN, HIGH,10000);
  distance = (duration * 0.034) / 2; 

  
  counter = counter + 1;
  if (counter >= 600){
    speed = 40;
  }

  
  if (distance >= seuil) {
    isSurfaceBelow = true;  // Vide détecté
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);

    
    return;     
  } else {
    isSurfaceBelow = false; // Pas de vide
  }

  
  bool left = digitalRead(CPT_LEFT);   // Capteur gauche
  bool right = digitalRead(CPT_RIGHT); // Capteur droit

  if (!left && !right) {
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  } else if (!left && right) {
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
  Serial.println(speed);
  delay(10);
}
