#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>
#include <Servo.h>
#include "utils.h"
#include <Adafruit_SSD1306.h>   // Assumed display library

// ================================================================
//                        Pin Definitions
// ================================================================

// OLED Display
#define OLED_RESET 1

// Motor control pins (shared between routines)
#define IN1 4 
#define IN2 5 
#define IN3 6
#define IN4 8

// Ultrasonic Sensor pins (from your original code)
#define CPT_US_CENTRAL_ECHO_PIN 26
#define CPT_US_CENTRAL_TRIG_PIN 27
// NOTE: In your sensors[] array you reference other sensor pins. Make sure they are defined if used.

// Encoder pins
#define ENC_LEFT_1 3
#define ENC_LEFT_2 2
#define ENC_RIGHT_1 19
#define ENC_RIGHT_2 18

// Relay (original code) – note: same as PIN_EJECTEUR in banner routine per instructions.
// #define RELAY 50

// ================================================================
//                 Banner Routine Pin Definitions
// ================================================================
#define PIN_PINCE_FERMETURE 48
#define PIN_VENTILLO 47
#define PIN_PINCE_OUVERTURE 46
#define PIN_SERVO 49
#define PIN_EJECTEUR 50
#define PIN_EJECTEUR_RETRACTION 51

// Motor speed for banner routine
#define MOTOR_SPEED 40

// ================================================================
//                     Global Variables & Objects
// ================================================================

// Servo for the banner routine
Servo myServo;

// OLED display – adjust parameters as needed for your hardware.
Adafruit_SSD1306 display(OLED_RESET);

// Speed settings for your normal movement (using encoder feedback)
volatile float speed = 80; // Good value is 80
const float offsetRightLeft = 112.5 / 100.0; // 112.5/100

// Timing for overall run (in seconds for stopAfterSec and delayStartSec)
const unsigned long stopAfterSec = 10000;
const unsigned long delayStartSec = 2;

// Ultrasonic obstacle threshold and reading interval (in ms)
const float obstacleThreshold = 15.0;
const int ultrasonicInterval = 50;

// For encoder-based movement
float tickrateByDegre = 29.722;

// Global flag to ensure the banner routine is executed only once.
bool bannerExecuted = false;

// ---------------------------------------------------------------
// Forward-declarations of Movement structure and UltrasonicSensor
// (as defined in utils.h)
// ---------------------------------------------------------------
// struct Movement{
//   long distance;
//   void (*movement)();
//   bool isEnd;
//   long position;
// };
// struct UltrasonicSensor {
//   int trigPin;
//   int echoPin;
//   int distance;
//   bool isNear;
// };

// ================================================================
//                      Function Prototypes
// ================================================================
void moveForward();
void moveBackward();
void turnLeft();
void turnRight180();
void turnLeft180();
void turnRight90();
void turnLeft90();
void deactivateUSSensor();
void activateUSSensor();

// FSM helper functions
void applyMovementSequence();
void stopMotors();
void resetEnc();
long averageTickrate();
long cmToTickrate(long centimeters);

// Ultrasonic sensor functions
float readUltrasonic(int trigPin, int echoPin);
void readUltrasonicSensors();
int checkUltrasonicSensors();
void updateUltrasonicReadings();

void deployBanner();
void dropBanner();

// ================================================================
//                     FSM State Definitions
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
//                   Global Variables (FSM)
// ================================================================
unsigned long startTime;
unsigned long lastUltrasonicCheck = 0;
bool currentSensor = false;
bool sensorOff = false;
int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);

// Placeholder distances for obstacles (updated by sensors)
float obstacleRightDistance = 0.0;
float obstacleLeftDistance = 0.0;
float obstacleCentralDistance = 0.0;

// Encoder objects
Encoder encLeft(ENC_LEFT_1, ENC_LEFT_2);
Encoder encRight(ENC_RIGHT_1, ENC_RIGHT_2);

// ================================================================
//        Define your UltrasonicSensor array (adjust if needed)
// ================================================================
UltrasonicSensor sensors[] = {
  {CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN, 100, false}  // Example sensor
};

int ultrasonicSensorCount = sizeof(sensors) / sizeof(sensors[0]);

// ================================================================
//         Movement Sequence Array (Banner routine removed)
// ================================================================
const Movement movementSequence[] = {
  {1000, deactivateUSSensor, false, 0},
  {6, moveForward, false, 0},
  {1000, deployBanner, false, 0},
  {8, moveBackward, false, 0},
  {1000, dropBanner, false, 0},
  {1000, activateUSSensor, false, 0},
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {60, moveForward, false, 0}
};
const int movementSequenceCount = sizeof(movementSequence) / sizeof(movementSequence[0]);
int movementSequenceNumber = 0;
Movement currentMovement = movementSequence[movementSequenceNumber];

// ================================================================
//                              Setup
// ================================================================
void setup() {
  Serial.begin(9600);
  
  // Initialize relay (used in normal FSM state) 
  // pinMode(RELAY, INPUT_PULLUP);
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Setup ultrasonic sensor pins (for each sensor in the array)
  for (int i = 0; i < ultrasonicSensorCount; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }
  
  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // -----------------------
  // Banner routine pin setup
  // -----------------------
  pinMode(PIN_PINCE_FERMETURE, OUTPUT);
  pinMode(PIN_VENTILLO, OUTPUT);
  pinMode(PIN_PINCE_OUVERTURE, OUTPUT);
  pinMode(PIN_EJECTEUR, OUTPUT);
  pinMode(PIN_EJECTEUR_RETRACTION, OUTPUT);
  myServo.attach(PIN_SERVO);

  myServo.write(10);
  digitalWrite(PIN_PINCE_FERMETURE, HIGH);
  delay(3000); // used to be TIME_1
  digitalWrite(PIN_PINCE_FERMETURE, LOW);
}

// ================================================================
//                              Loop (FSM)
// ================================================================
void loop() {
  unsigned long elapsedTime = (millis() - startTime) / 100; // elapsed time in tenths of seconds
  updateScore(elapsedTime);
  readUltrasonicSensors();
  unsigned int sensorDetect = checkUltrasonicSensors();
    
  switch (currentState) {
    case IDLE:
      // if (digitalRead(RELAY) == HIGH) {
      //   Serial.println("tirette");
      //   startTime = millis();
      //   currentState = START;
      // }
      startTime = millis();
      currentState = START;
      break;
      
    case START:
      // Wait for delay before transitioning to RUNNING
      if (elapsedTime >= delayStartSec * 10) {
        currentState = RUNNING;
      }
      break;

    case RUNNING:
      if (sensorDetect != 1000 && !sensorOff) {
        currentState = AVOID_OBSTACLE;
      } else if (elapsedTime >= (maxTime / 100)) {
        currentState = STOPPED;
      } else {
        applyMovementSequence();
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
//                   FSM Helper Function Definitions
// ================================================================
void applyMovementSequence(){
  if (movementSequenceNumber >= movementSequenceCount) {
    stopMotors();
    return;
  }
  
  if (currentMovement.isEnd) {
    currentMovement = movementSequence[++movementSequenceNumber];
    stopMotors();
    delay(1500);
    resetEnc();
  } else {
    currentMovement.movement();
  }
}

void resetEnc(){
  encLeft.write(0);
  encRight.write(0);
}

long averageTickrate(){
  return (encLeft.read() + encRight.read()) / 2;
}

long cmToTickrate(long centimeters){ 
  return centimeters * 200;
}

// -----------------------------------------------------
// Standard encoder-based moveForward
// -----------------------------------------------------
void moveForward() {
  currentMovement.position = averageTickrate();
  if (currentMovement.position >= cmToTickrate(currentMovement.distance)) {
    currentMovement.isEnd = true;
    return;
  }
  analogWrite(IN1, (int)(speed * offsetRightLeft));
  digitalWrite(IN2, LOW);
  analogWrite(IN3, (int)speed);
  digitalWrite(IN4, LOW);
}
// -----------------------------------------------------
// Standard encoder-based moveBackward
// -----------------------------------------------------
void moveBackward() {
  currentMovement.position = averageTickrate();
  if (currentMovement.position <= -cmToTickrate(currentMovement.distance)) {
    currentMovement.isEnd = true;
    return;
  }
  digitalWrite(IN1, LOW);
  analogWrite(IN2, (int)(speed * offsetRightLeft));
  digitalWrite(IN3, LOW);
  analogWrite(IN4, (int)speed);
}

// -----------------------------------------------------
// Motor turning functions using encoder ticks
// -----------------------------------------------------
void turnRight180() {
  float tickrateTurn = 5500;
  long tickrateLeft = encLeft.read();
  long tickrateRight = abs(encRight.read());
  if (tickrateLeft <= tickrateTurn) {
    analogWrite(IN1, (int)(speed * offsetRightLeft));
    digitalWrite(IN2, LOW);
  }
  if (tickrateRight <= tickrateTurn) {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, (int)speed);
  }
  currentMovement.isEnd = (tickrateLeft >= tickrateTurn && tickrateRight >= tickrateTurn);
}

void turnLeft180() {
  float tickrateTurn = 5500;
  long tickrateLeft = abs(encLeft.read());
  long tickrateRight = encRight.read();
  if (tickrateLeft <= tickrateTurn) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, (int)(speed * offsetRightLeft));
  }
  if (tickrateRight <= tickrateTurn) {
    analogWrite(IN3, (int)speed); 
    digitalWrite(IN4, LOW);
  }
  currentMovement.isEnd = (tickrateLeft >= tickrateTurn && tickrateRight >= tickrateTurn);
}

void turnRight90() {
  float tickrateTurn = 2400;
  long tickrateLeft = encLeft.read();
  long tickrateRight = abs(encRight.read());
  if (tickrateLeft <= tickrateTurn) {
    analogWrite(IN1, (int)(speed * offsetRightLeft));
    digitalWrite(IN2, LOW);
  }
  if (tickrateRight <= tickrateTurn) {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, (int)speed);
  }
  currentMovement.isEnd = (tickrateLeft >= tickrateTurn && tickrateRight >= tickrateTurn);
}

void turnLeft90() {
  float tickrateTurn = 2400;
  long tickrateLeft = abs(encLeft.read());
  long tickrateRight = encRight.read();
  if (tickrateLeft <= tickrateTurn) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, (int)(speed * offsetRightLeft));
  }
  if (tickrateRight <= tickrateTurn) {
    analogWrite(IN3, (int)speed); 
    digitalWrite(IN4, LOW);
  }
  currentMovement.isEnd = (tickrateLeft >= tickrateTurn && tickrateRight >= tickrateTurn);
}

// -----------------------------------------------------
// Simple left turn without encoder feedback
// -----------------------------------------------------
void turnLeft() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, (int)(speed * offsetRightLeft));
  analogWrite(IN3, (int)speed);
  digitalWrite(IN4, LOW);
}

// -----------------------------------------------------
// Stop all motors
// -----------------------------------------------------
void stopMotors() {
  Serial.println("stopMotors");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// -----------------------------------------------------
// Ultrasonic functions
// -----------------------------------------------------
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 10000);
  return (duration * 0.0343) / 2; 
}

void readUltrasonicSensors() {
  for (int i = 0; i < ultrasonicSensorCount; i++) {    
    digitalWrite(sensors[i].trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensors[i].trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensors[i].trigPin, LOW);
    long obstacle_duration = pulseIn(sensors[i].echoPin, HIGH, 3000);
    if (obstacle_duration == 0) {
      sensors[i].distance = 100;
    } else {
      sensors[i].distance = obstacle_duration * 0.034 / 2;
    }
    sensors[i].isNear = (sensors[i].distance <= obstacleThreshold);
  }
}

int checkUltrasonicSensors(){
  for (int i = 0; i < ultrasonicSensorCount; i++) {
    if (sensors[i].isNear) {
      return i;
    }
  }
  return 1000;
}

void updateUltrasonicReadings() {
  unsigned long currentTime = millis();
  if (currentTime - lastUltrasonicCheck >= ultrasonicInterval) {
    lastUltrasonicCheck = currentTime;
    if (currentSensor) {
      obstacleLeftDistance = readUltrasonic(CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN);
    } else {
      obstacleRightDistance = readUltrasonic(CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN);
    }
    currentSensor = !currentSensor;
  }
}

// -----------------------------------------------------
// Score update function (for OLED)
// -----------------------------------------------------
void updateScore(unsigned long time) {
  int score;
  const char* smiley;
  if (time > 20) {
    score = 45;
  } else if (time > 14) {
    score = 35;
  } else if (time > 4) {
    score = 20;
  } else {
    score = 0;
  }

  if ((time / 8) % 2) {
    smiley = "(>'-')> SCORE <('-'<)";
  } else {
    smiley = "<('-'<) SCORE (>'-')>";
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(smiley);
  display.setTextSize(2);
  display.setCursor(50, 15);
  display.println(score);
  display.display();
}

// -----------------------------------------------------
// Stub functions for sensor control
// -----------------------------------------------------
void deactivateUSSensor() {
  sensorOff = true;
  currentMovement.isEnd = true;
}

void activateUSSensor() {
  sensorOff = false;
  currentMovement.isEnd = true;
}

// ================================================================
//               Banner Routine Implementation
// ================================================================

// /*
//   bannerRoutine()
  
//   This function encapsulates the entire banner sequence using fixed delays
//   with raw integer values. It is called once (before any other action)
//   and does not repeat.
// */
// void bannerRoutine() {
//   // Step 1: Initialize servo position
//   myServo.write(10);
  
//   // Step 2: Activate fermeture pince
//   digitalWrite(PIN_PINCE_FERMETURE, HIGH);
//   delay(3000); // used to be TIME_1
//   digitalWrite(PIN_PINCE_FERMETURE, LOW);
  
//   // Step 3: Move servo to new position for repositioning
//   myServo.write(120);
//   delay(1000);  // stabilization delay
  
//   // Step 4: Advance forward (timed)
//   currentMovement.distance = 10;
//   moveForward();
//   delay(1500); // used to be TIME_3
//   bannerStopMotors();
  
//   // Step 5: Activate ventilo
//   digitalWrite(PIN_VENTILLO, HIGH);
//   delay(4000); // used to be TIME_4
  
//   // Step 6: Reverse backward (timed)
//   bannerMoveBackward();
//   delay(4000); // used to be TIME_2
//   bannerStopMotors();
  
//   // Step 7: Advance very slightly
//   bannerMoveForward();
//   delay(1000); // used to be TIME_6
//   bannerStopMotors();
  
//   // Step 8: Deactivate ventilo
//   digitalWrite(PIN_VENTILLO, LOW);
  
//   // Step 9: Activate pince ouverture
//   digitalWrite(PIN_PINCE_OUVERTURE, HIGH);
//   delay(4000); // used to be TIME_5
//   digitalWrite(PIN_PINCE_OUVERTURE, LOW);
  
//   // Step 10: Activate ejecteur
//   digitalWrite(PIN_EJECTEUR, HIGH);
//   delay(2200); // used to be TIME_7
//   digitalWrite(PIN_EJECTEUR, LOW);
  
//   // Step 11: Reset section - reposition servo and retract ejecteur
//   myServo.write(10);
//   digitalWrite(PIN_EJECTEUR_RETRACTION, HIGH);
//   delay(2200); // used to be TIME_7
//   digitalWrite(PIN_EJECTEUR_RETRACTION, LOW);
  
//   // Banner routine complete; control returns to the FSM.
// }
void deployBanner() {  
  // Step 3: Move servo to new position for repositioning
  myServo.write(120);
  delay(1000);  // stabilization delay
  digitalWrite(PIN_VENTILLO, HIGH);
  currentMovement.isEnd = true;
}

void dropBanner() {
  // Step 8: Deactivate ventilo
  digitalWrite(PIN_VENTILLO, LOW);
  
  // Step 9: Activate pince ouverture
  digitalWrite(PIN_PINCE_OUVERTURE, HIGH);
  delay(4000); // used to be TIME_5
  digitalWrite(PIN_PINCE_OUVERTURE, LOW);
  
  // Step 10: Activate ejecteur
  digitalWrite(PIN_EJECTEUR, HIGH);
  delay(2200); // used to be TIME_7
  digitalWrite(PIN_EJECTEUR, LOW);
  
  // Step 11: Reset section - reposition servo and retract ejecteur
  myServo.write(10);
  digitalWrite(PIN_EJECTEUR_RETRACTION, HIGH);
  delay(2200); // used to be TIME_7
  digitalWrite(PIN_EJECTEUR_RETRACTION, LOW);
  currentMovement.isEnd = true;
}