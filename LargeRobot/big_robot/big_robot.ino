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

// Forward Ultrasonic Sensor pins
#define CPT_US_RIGHT_TRIG_PIN 35
#define CPT_US_RIGHT_ECHO_PIN 34 
#define CPT_US_LEFT_TRIG_PIN 31
#define CPT_US_LEFT_ECHO_PIN 30
#define CPT_US_CENTRAL_TRIG_PIN 27
#define CPT_US_CENTRAL_ECHO_PIN 26
// #define CPT_US_BACK_RIGHT_TRIG_PIN 38
// #define CPT_US_BACK_RIGHT_ECHO_PIN 39
// #define CPT_US_BACK_LEFT_TRIG_PIN 40
// #define CPT_US_BACK_LEFT_ECHO_PIN 41
// #define CPT_US_BACK_CENTRAL_TRIG_PIN 28
// #define CPT_US_BACK_CENTRAL_ECHO_PIN 29

// Encoder pins
#define ENC_LEFT_1 3
#define ENC_LEFT_2 2
#define ENC_RIGHT_1 19
#define ENC_RIGHT_2 18

//Relay
#define RELAY 24 //TODO: changer la pin de la tirette.

//Switches
#define SWITCH_MSB 52
#define SWITCH_LSB 53

//Banner Routine Pin Definitions
#define PIN_PINCE_FERMETURE 48
#define PIN_VENTILLO 47
#define PIN_PINCE_OUVERTURE 46
#define PIN_SERVO 49
#define PIN_EJECTEUR 50
#define PIN_EJECTEUR_RETRACTION 51
#define MOTOR_SPEED 40 //TODO: mettre dans parametres

// ================================================================
//                    Initialisation functions
// ================================================================

void moveForward();
void moveBackward();
void turnRight180();
void turnLeft180();
void turnRight90();
void turnLeft90();

void deactivateUSSensor();
void activateUSSensor();

void deployBanner();
void dropBanner();

// ================================================================
//                           Parameters
// ================================================================

// generalSpeed settings for general normal movement (using encoder feedback)
volatile float generalSpeed = 80; //Good value is 80
const float offsetRightLeft = 112.5/100; // 112.5/100

// Timing for overall run (in seconds)
const unsigned long stopAfterSec = 100;
const unsigned long delayStartSec = 5;

// Ultrasonic obstacle threshold and reading interval (in ms)
const float obstacleThreshold = 15.0;
const int ultrasonicInterval = 50;

// For encoder-based movement
float tickrateByDegre = 29.722;

// ================================================================
//                       Movements sequences
// ================================================================

const Movement movementSequence[] = {
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {40, moveForward, false, 0},
  {1000, turnLeft90, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {60, moveBackward, false, 0},
  {40, moveForward, false, 0},
};

// const Movement movementSequence[] = {
//   {1000, deactivateUSSensor, false, 0},
//   {6, moveForward, false, 0},
//   {1000, deployBanner, false, 0},
//   {8, moveBackward, false, 0},
//   {1000, dropBanner, false, 0},
//   {1000, activateUSSensor, false, 0},
//   {40, moveForward, false, 0},
//   {1000, turnRight90, false, 0},
//   {40, moveForward, false, 0},
//   {1000, turnRight90, false, 0},
//   {1000, deactivateUSSensor, false, 0},
//   {60, moveForward, false, 0}
// };

const int movementSequenceCount = sizeof(movementSequence) / sizeof(movementSequence[0]);
int movementSequenceNumber = 0;
Movement currentMovement = movementSequence[movementSequenceNumber];

// ================================================================
//                           Initialisation
// ================================================================

const float speedRight = generalSpeed;
const float speedLeft = generalSpeed * offsetRightLeft;

bool sensorOff = false;
int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);

unsigned int startTime;

UltrasonicSensor sensors[] = {
  {CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN, 100, false},
  {CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN, 100, false},
  {CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN, 100, false},
};
int ultrasonicSensorCount = sizeof(sensors) / sizeof(sensors[0]);

Encoder encLeft(ENC_LEFT_1, ENC_LEFT_2);
Encoder encRight(ENC_RIGHT_1, ENC_RIGHT_2);

// Servo for the banner routine
Servo myServo;

// OLED display – adjust parameters as needed for your hardware.
Adafruit_SSD1306 display(OLED_RESET);

// Global flag to ensure the banner routine is executed only once.
bool bannerExecuted = false;

int MSB;
int LSB;

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

  // Initialize relay
  pinMode(RELAY, INPUT_PULLUP);

  pinMode(SWITCH_MSB, INPUT);
  pinMode(SWITCH_LSB, INPUT);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  for(int i = 0; i < ultrasonicSensorCount; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // -----------------------
  // Banner routine pin setup
  // -----------------------
  // pinMode(PIN_PINCE_FERMETURE, OUTPUT);
  // pinMode(PIN_VENTILLO, OUTPUT);
  // pinMode(PIN_PINCE_OUVERTURE, OUTPUT);
  // pinMode(PIN_EJECTEUR, OUTPUT);
  // pinMode(PIN_EJECTEUR_RETRACTION, OUTPUT);
  // myServo.attach(PIN_SERVO);

  // myServo.write(10);
  // digitalWrite(PIN_PINCE_FERMETURE, HIGH);
  // delay(3000); // used to be TIME_1
  // digitalWrite(PIN_PINCE_FERMETURE, LOW);
}

// ================================================================
//                           Loop (FSM Logic)
// ================================================================
void loop() {
  unsigned long elapsedTime = millis() - startTime; // Temps écoulé en DIXIEMES de secondes
  updateScore(elapsedTime);
  readUltrasonicSensors();

  // selectSequences();

  unsigned int sensorDetect = checkUltrasonicSensors();
    
  switch (currentState) {
    
    case IDLE:
      if(digitalRead(RELAY) == HIGH){
        // Serial.println("IDLE");
        startTime = millis();
        currentState = START;
      }
      break;
      
    case START:
      if (elapsedTime * 1000 >= delayStartSec) {
        // Serial.println("START");
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      if (sensorDetect != 1000 && !sensorOff) {
        currentState = AVOID_OBSTACLE;
      } else if (elapsedTime >= (maxTime / 1000)) {
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
//                           Functions
// ================================================================

void applyMovementSequence(){
  if (movementSequenceNumber >= movementSequenceCount) {
    stopMotors();
    return;
  }
  
  if (currentMovement.isEnd) {
    currentMovement = movementSequence[++movementSequenceNumber];
    stopMotors();
    delay(1000);
    resetEnc();
  } else {
    currentMovement.movement();
  }
}

// -----------------------------------------------------
// Encoder functions
// -----------------------------------------------------
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
// Ultrasonic functions
// -----------------------------------------------------
void deactivateUSSensor(){
  sensorOff = true;
  currentMovement.isEnd = true;
}

void activateUSSensor(){
  sensorOff = false;
  currentMovement.isEnd = true;
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

/**
 * @brief Mesure la distance avec tous les capteurs ultrason.
 *
 * Pour chaque capteur, envoie une impulsion et mesure la durée de l'écho afin de calculer la distance.
 * Met à jour le champ distance et le booléen isNear pour chaque capteur.
 */
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
    }
    else {
      sensors[i].distance = obstacle_duration * 0.034 / 2;
    }
    sensors[i].isNear = (sensors[i].distance <= obstacleThreshold);
  }
}

int checkUltrasonicSensors(){
  for (int i = 0; i < ultrasonicSensorCount; i++) {
    if(sensors[i].isNear){
      return i;
    }
  }
  return 1000;
}

// -----------------------------------------------------
// Movement functions
// -----------------------------------------------------
/**
 * @brief Fait avancer le robot.
 *
 * Active les moteurs pour un mouvement en avant.
 */
void moveForward() {
  Serial.println("moveForward");
  currentMovement.position = averageTickrate();
  if (currentMovement.position >= cmToTickrate(currentMovement.distance)) {
    currentMovement.isEnd = true;
    return;
  }
  analogWrite(IN1, speedLeft);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, speedRight);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  currentMovement.position = averageTickrate();
  if (currentMovement.position <= -cmToTickrate(currentMovement.distance)) {
    currentMovement.isEnd = true;
    return;
  }
  digitalWrite(IN1, LOW);
  analogWrite(IN2, speedLeft);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, speedRight);
}

/**
 * @brief Fait tourner le robot vers la gauche.
 *
 * Active les moteurs pour réaliser une rotation vers la gauche.
 */
void turnRight180() {
  float tirckrateTurn =  5500;
  long tickrateLeft = encLeft.read();
  long tickrateRight = abs(encRight.read());
  if(tickrateLeft <= tirckrateTurn){
    analogWrite(IN1, speedLeft);
    digitalWrite(IN2, LOW);
  }
  if(tickrateRight <= tirckrateTurn) {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, speedRight);
  }
  currentMovement.isEnd = (tickrateLeft >= tirckrateTurn && tickrateRight >= tirckrateTurn);
}

void turnLeft180() {
  float tirckrateTurn =  5500;
  long tickrateLeft = abs(encLeft.read());
  long tickrateRight = encRight.read();
  if(tickrateLeft <= tirckrateTurn){
    digitalWrite(IN1, LOW);
    analogWrite(IN2, speedLeft);
  }
  if(tickrateRight <= tirckrateTurn) {
    analogWrite(IN3, speedRight); 
    digitalWrite(IN4, LOW);
  }
  currentMovement.isEnd = (tickrateLeft >= tirckrateTurn && tickrateRight >= tirckrateTurn);
}

void turnRight90() {
  float tirckrateTurn =  2400;
  long tickrateLeft = encLeft.read();
  long tickrateRight = abs(encRight.read());
  if(tickrateLeft <= tirckrateTurn){
    analogWrite(IN1, speedLeft);
    digitalWrite(IN2, LOW);
  }
  if(tickrateRight <= tirckrateTurn) {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, speedRight);
  }
  currentMovement.isEnd = (tickrateLeft >= tirckrateTurn && tickrateRight >= tirckrateTurn);
}

void turnLeft90() {
  float tirckrateTurn =  2400;
  long tickrateLeft = abs(encLeft.read());
  long tickrateRight = encRight.read();
  if(tickrateLeft <= tirckrateTurn){
    digitalWrite(IN1, LOW);
    analogWrite(IN2, speedLeft);
  }
  if(tickrateRight <= tirckrateTurn) {
    analogWrite(IN3, speedRight); 
    digitalWrite(IN4, LOW);
  }
  currentMovement.isEnd = (tickrateLeft >= tirckrateTurn && tickrateRight >= tirckrateTurn);
}

/**
 * @brief Arrête tous les moteurs du robot.
 *
 * Met les pins des moteurs à LOW pour arrêter tout mouvement.
 */
void stopMotors() {
  Serial.println("stopMotors");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// -----------------------------------------------------
// Score update function (for OLED)
// -----------------------------------------------------
/**
 * @brief Met à jour l'affichage du score en fonction du temps écoulé.
 *
 * Détermine un score à afficher selon la valeur du temps donné.
 * Affiche un smiley ASCII différent selon la parité de `time`, 
 * puis affiche le score correspondant sur l'écran OLED.
 * Le score est affiché en grand au centre de l'écran.
 *
 * @param time Le temps écoulé en dixièmes de secondes, utilisé pour déterminer le score.
 */
void updateScore(unsigned long time) {
  int score;
  long sec = 1000;
  const char* smiley;
  if (time > 20*sec) {
    score = 45;
  } else if (time > 14*sec) {
    score = 35;
  } else if (time > 4*sec) {
    score = 20;
  } else {
    score = 0;
  }

  if ((time/800)%2) {
    smiley = "(>'-')> SCORE <('-'<)";
  } else {
    smiley = "<('-'<) SCORE (>'-')>";
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(smiley);
  display.setTextSize(2);
  display.setCursor(50,15);
  display.println(score);
  display.display();
}

// -----------------------------------------------------
// Banner Routine Implementation
// -----------------------------------------------------
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

int selectSequences(){
  int valeur = 0;
  if (digitalRead(SWITCH_MSB) == 1) {
    valeur += 2;
  }
  if (digitalRead(SWITCH_LSB) == 1) {
    valeur += 1;
  }

  switch(valeur){
    case 0: 
      Serial.println(0);
      break;
    case 1: 
      Serial.println(1);
      break;
    case 2: 
      Serial.println(2);
      break;
    case 3: 
      Serial.println(3);
      break;
  }
}
