#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

Servo myServo;            
const int SERVO_PIN = 5;  

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

#define CPT_LEFT 7
#define CPT_RIGHT 8
#define CPT_US_TRIG_PIN 12
#define CPT_US_ECHO_PIN 9

volatile int speed = 65;
float distance = 0.0;
const float seuil = 7.0;  
bool isSurfaceBelow = false;

volatile int counter = 1;

const int LED_INTERVAL = 300;     
const int SERVO_MIN_ANGLE = 0;    
const int SERVO_MAX_ANGLE = 180;  
unsigned long spinTime = 5000;    

void setup() {
  pinMode(13, OUTPUT);
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);
  pinMode(CPT_US_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_ECHO_PIN, INPUT);


  AFMS.begin();

  
  myServo.attach(SERVO_PIN);
  myServo.write(90);  

  Serial.begin(9600);               
  digitalWrite(LED_BUILTIN, HIGH);  
}

void loop() {
  digitalWrite(CPT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_TRIG_PIN, LOW);

  long duration = pulseIn(CPT_US_ECHO_PIN, HIGH, 10000);
  distance = (duration * 0.034) / 2;

  if (distance >= seuil) {
    isSurfaceBelow = true;
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    randomSpin();
    return;
  } else {
    isSurfaceBelow = false;
  }

  counter++;
  if (counter >= 600) {
    speed = 45;
    counter = 0;
  }

  bool left = digitalRead(CPT_LEFT);
  bool right = digitalRead(CPT_RIGHT);

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

}

void randomSpin() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  unsigned long startTime = millis();  
  unsigned long lastBlinkTime = startTime;

  digitalWrite(LED_BUILTIN, HIGH);

  while (millis() - startTime < spinTime) {
    if (millis() - lastBlinkTime >= LED_INTERVAL) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      lastBlinkTime = millis();
    }

    int randomAngle = random(SERVO_MIN_ANGLE, SERVO_MAX_ANGLE + 1);
    myServo.write(randomAngle);  
    delay(500);                  
  }

  myServo.write(90);
  digitalWrite(LED_BUILTIN, LOW);

}
