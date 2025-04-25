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
#define CPT_US_BACK_RIGHT_TRIG_PIN 39
#define CPT_US_BACK_RIGHT_ECHO_PIN 38
#define CPT_US_BACK_LEFT_TRIG_PIN 41
#define CPT_US_BACK_LEFT_ECHO_PIN 40
#define CPT_US_BACK_CENTRAL_TRIG_PIN 28
#define CPT_US_BACK_CENTRAL_ECHO_PIN 29


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
void waitPAMIs();

void deactivateUSSensor();
void activateUSSensor();

void checkEncoderOn();
void checkEncoderOff();

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
const float obstacleThreshold = 12.0;
const int ultrasonicInterval = 50;

// For encoder-based movement
float tickrateByDegre = 29.722;

// Timeout
const unsigned long blockTimeout = 2000;

// For delaying score update
const unsigned long interval = 500;
unsigned long lastUpdateTime = 0;

unsigned long timingPAMIs = 94000;
unsigned long elapsedTime = 0; // Temps écoulé en millièmes de secondes

// ================================================================
//                       Movements sequences
// ================================================================

// Movement choice at the beginning
bool movementCreated = false;
const Movement blueMovementSequence[] = {
  {2, moveForward, false, 0},
  {1000, deployBanner, false, 0},
  {3, moveBackward, false, 0},
  {1000, dropBanner, false, 0},
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {40, moveForward, false, 0},
  {1000, turnLeft90, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {1000, checkEncoderOn, false, 0},
  {100, moveBackward, false, 0},
  {1000, checkEncoderOff, false, 0},
  {10, moveForward, false, 0},
  {25, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {1000, activateUSSensor, false, 0},
  {50, moveForward, false, 0},
  {1000, checkEncoderOn, false, 0},
  {100, moveBackward, false, 0},
  {1000, checkEncoderOff, false, 0},
  {1000, activateUSSensor, false, 0},
  {60, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {30, moveForward, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {1000, turnRight90, false, 0},
  {1000, waitPAMIs, false, 0},
  {95, moveBackward, false, 0},
};

const Movement yellowMovementSequence[] = {
  {2, moveForward, false, 0},
  {1000, deployBanner, false, 0},
  {3, moveBackward, false, 0},
  {1000, dropBanner, false, 0},
  {40, moveForward, false, 0},
  {1000, turnLeftt90, false, 0},
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {1000, checkEncoderOn, false, 0},
  {100, moveBackward, false, 0},
  {1000, checkEncoderOff, false, 0},
  {10, moveForward, false, 0},
  {25, moveForward, false, 0},
  {1000, turnLeft90, false, 0},
  {1000, activateUSSensor, false, 0},
  {50, moveForward, false, 0},
  {1000, checkEncoderOn, false, 0},
  {100, moveBackward, false, 0},
  {1000, checkEncoderOff, false, 0},
  {1000, activateUSSensor, false, 0},
  {60, moveForward, false, 0},
  {1000, turnLeft90, false, 0},
  {30, moveForward, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {1000, turnLeft90, false, 0},
  {1000, waitPAMIs, false, 0},
  {95, moveBackward, false, 0},
};

const Movement* movementSequence = nullptr;
Movement currentMovement;

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

const int movementSequenceCount = sizeof(blueMovementSequence) / sizeof(blueMovementSequence[0]);
int movementSequenceNumber = 0;

// ================================================================
//                           Initialisation
// ================================================================

const float speedRight = generalSpeed;
const float speedLeft = generalSpeed * offsetRightLeft;

bool sensorOff = true;
int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);

unsigned int startTime;

UltrasonicSensor frontSensors[] = {
  {CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN, 100, false},
  {CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN, 100, false},
  {CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN, 100, false},
};
int frontUltrasonicSensorCount = sizeof(frontSensors) / sizeof(frontSensors[0]);

UltrasonicSensor backSensors[]{
  {CPT_US_BACK_RIGHT_TRIG_PIN, CPT_US_BACK_RIGHT_ECHO_PIN, 100, false},
  {CPT_US_BACK_LEFT_TRIG_PIN, CPT_US_BACK_LEFT_ECHO_PIN, 100, false},
  {CPT_US_BACK_CENTRAL_TRIG_PIN, CPT_US_BACK_CENTRAL_ECHO_PIN, 100, false},
};
int backUltrasonicSensorCount = sizeof(backSensors) / sizeof(backSensors[0]);

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

bool checkEncoder = false;
long lastLeftEncoder = 0;
long lastRightEncoder = 0;
unsigned long lastEncoderChangeTime = 0;

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
  selectSequence();

  currentMovement = choosedSequence[choosedSequenceNumber];

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  for(int i = 0; i < frontUltrasonicSensorCount; i++) {
    pinMode(frontSensors[i].trigPin, OUTPUT);
    pinMode(frontSensors[i].echoPin, INPUT);
  }
  for (int i = 0; i < backUltrasonicSensorCount; i++){
    pinMode(backSensors[i].trigPin, OUTPUT);
    pinMode(backSensors[i].echoPin, INPUT);
  }

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  
  // Sequence selection (strategy)
  selectSequence();





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
  if (digitalRead(RELAY) == LOW) {
    static bool messageShown = false;
    if (!messageShown) {
      displayWaitingMessage();
      messageShown = true;
    }
  }

  selectSequences();
  isRobotBlocked();

  // Serial.println(getMovementName(currentMovement.movement));

  unsigned int frontSensorDetect = 1000;
  unsigned int backSensorDetect = 1000;
    
  switch (currentState) {
    
    case IDLE:
      if(digitalRead(RELAY) == HIGH){
        // Serial.println("IDLE");
        startTime = millis();
        currentState = START;
      } else
      break;
      
    case START:
      if (elapsedTime * 1000 >= delayStartSec) {
        // Serial.println("START");
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      if (currentMovement.movement != moveBackward) {
        readFrontUltrasonicSensors();
        frontSensorDetect = checkFrontUltrasonicSensors();
      }
      if (currentMovement.movement != moveForward){
        readBackUltrasonicSensors();
        // backSensorDetect = checkBackUltrasonicSensors();
        backSensorDetect = 1000;
      }
      if (((backSensorDetect != 1000) || (frontSensorDetect != 1000)) && !sensorOff) {
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
const char* getMovementName(void (*movement)()) {
  if (movement == moveForward) return "moveForward";
  if (movement == moveBackward) return "moveBackward";
  if (movement == turnLeft90) return "turnLeft90";
  if (movement == turnRight90) return "turnRight90";
  if (movement == turnLeft180) return "turnLeft180";
  if (movement == turnRight180) return "turnRight180";
  if (movement == activateUSSensor) return "activateUSSensor";
  if (movement == deactivateUSSensor) return "deactivateUSSensor";
  if (movement == deployBanner) return "deployBanner";
  if (movement == dropBanner) return "dropBanner";
  return "Unknown";
}


void applyMovementSequence(){
  if (choosedSequenceNumber  >= choosedSequenceCount ) {
    stopMotors();
    return;
  }
  
  if (currentMovement.isEnd) {
    currentMovement = choosedSequence[++choosedSequenceNumber];
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

void checkEncoderOn(){
  checkEncoder = true;
  currentMovement.isEnd = true;
}

void checkEncoderOff(){
  checkEncoder = false;
  currentMovement.isEnd = true;
}

void isRobotBlocked(){
   if(checkEncoder) {
    // Serial.println(checkEncoder);
    long currentLeft = encLeft.read();
    long currentRight = encRight.read();

    // Vérifie si les encodeurs ont changé
    if (abs(currentLeft - lastLeftEncoder) > 5 || abs(currentRight - lastRightEncoder) > 5) {
      // Serial.println("Maj encoder");
      lastEncoderChangeTime = millis(); // Met à jour le dernier moment de mouvement
      lastLeftEncoder = currentLeft;
      lastRightEncoder = currentRight;
    } 
    if (millis() - lastEncoderChangeTime > blockTimeout) {
      // Serial.println("Change");
      currentMovement.isEnd = true;

      // Met à jour l'état pour éviter répétition
      lastEncoderChangeTime = millis(); 
      lastLeftEncoder = currentLeft;
      lastRightEncoder = currentRight;
    }
  }
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
void readBackUltrasonicSensors() {
  for (int i = 0; i < backUltrasonicSensorCount; i++) {    
    digitalWrite(backSensors[i].trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(backSensors[i].trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(backSensors[i].trigPin, LOW);
    long obstacle_duration = pulseIn(backSensors[i].echoPin, HIGH, 3000);
    if (obstacle_duration == 0) {
      backSensors[i].distance = 100;
    }
    else {
      backSensors[i].distance = obstacle_duration * 0.034 / 2;
    }
    backSensors[i].isNear = (backSensors[i].distance <= obstacleThreshold);
  }
}
void readFrontUltrasonicSensors() {
  for (int i = 0; i < frontUltrasonicSensorCount; i++) {    
    digitalWrite(frontSensors[i].trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(frontSensors[i].trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(frontSensors[i].trigPin, LOW);
    long obstacle_duration = pulseIn(frontSensors[i].echoPin, HIGH, 3000);
    if (obstacle_duration == 0) {
      frontSensors[i].distance = 100;
    }
    else {
      frontSensors[i].distance = obstacle_duration * 0.034 / 2;
    }
    frontSensors[i].isNear = (frontSensors[i].distance <= obstacleThreshold);
  }
}

int checkAllUltrasonicSensors(){
  int i = 1000;
  i = checkFrontUltrasonicSensors();
  if (i != 1000){
    i = checkBackUltrasonicSensors();
  }
  return i;
}

int checkFrontUltrasonicSensors(){
  for (int i = 0; i < frontUltrasonicSensorCount; i++) {
    if(frontSensors[i].isNear){
      return i;
    }
  }
  return 1000;
}

int checkBackUltrasonicSensors(){
  for (int i = 0; i < backUltrasonicSensorCount; i++) {
    if(backSensors[i].isNear){
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

// void moveForward() {
//   Serial.println("moveForward");
//   currentMovement.position = averageTickrate();
//   long distance = cmToTickrate(currentMovement.distance);
//   if (currentMovement.position >= distance) {
//     currentMovement.isEnd = true;
//     return;
//   }
//   if(encLeft.read() <= distance) {
//     analogWrite(IN1, speedLeft);
//     digitalWrite(IN2, LOW);
//   }
//   if(encRight.read() <= distance) {
//     analogWrite(IN3, speedRight);
//     digitalWrite(IN4, LOW);
//   }
// }

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
  const long tickTarget = 2420;
  const long slowDownThreshold = 400; // Distance avant la cible pour commencer à ralentir
  const int minSpeed = 80;            // Vitesse minimale pour garantir le mouvement

  long tickLeft = encLeft.read();
  long tickRight = abs(encRight.read());

  int adjustedSpeedLeft = speedLeft;
  int adjustedSpeedRight = speedRight;

  // Réduction de la vitesse à l’approche de la cible
  if ((tickTarget - tickLeft) <= slowDownThreshold) {
    float factor = float(tickTarget - tickLeft) / slowDownThreshold;
    adjustedSpeedLeft = max(int(speedLeft * factor), minSpeed);
  }

  if ((tickTarget - tickRight) <= slowDownThreshold) {
    float factor = float(tickTarget - tickRight) / slowDownThreshold;
    adjustedSpeedRight = max(int(speedRight * factor), minSpeed);
  }

  // Contrôle des moteurs
  if (tickLeft < tickTarget) {
    analogWrite(IN1, adjustedSpeedLeft);
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(IN1, 0);
    digitalWrite(IN2, LOW);
  }

  if (tickRight < tickTarget) {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, adjustedSpeedRight);
  } else {
    digitalWrite(IN3, LOW);
    analogWrite(IN4, 0);
  }

  // Mouvement terminé ?
  currentMovement.isEnd = (tickLeft >= tickTarget && tickRight >= tickTarget);
}


void turnLeft90() {
  const long tickTarget = 2420;
  const long slowDownThreshold = 400;
  const int minSpeed = 80;

  long tickLeft = abs(encLeft.read());
  long tickRight = encRight.read();

  int adjustedSpeedLeft = speedLeft;
  int adjustedSpeedRight = speedRight;

  Serial.print("tickLeft: "); Serial.print(tickLeft);
  Serial.print(" | tickRight: "); Serial.println(tickRight);
  // Ralentissement progressif côté gauche
  if ((tickTarget - tickLeft) <= slowDownThreshold) {
    float factor = float(tickTarget - tickLeft) / slowDownThreshold;
    adjustedSpeedLeft = max(int(speedLeft * factor), minSpeed);
  }

  // Ralentissement progressif côté droit
  if ((tickTarget - tickRight) <= slowDownThreshold) {
    float factor = float(tickTarget - tickRight) / slowDownThreshold;
    adjustedSpeedRight = max(int(speedRight * factor), minSpeed);
  }

  // Commandes moteurs avec vitesses ajustées
  if (tickLeft < tickTarget) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, adjustedSpeedLeft);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, 0);
  }

  if (tickRight < tickTarget) {
    analogWrite(IN3, adjustedSpeedRight);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(IN3, 0);
    digitalWrite(IN4, LOW);
  }
  
  currentMovement.isEnd = (tickLeft >= tickTarget && tickRight >= tickTarget);
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
void updateScore() {
  unsigned long time = millis() - startTime;
  int score;
  long sec = 1000;
  const char* smiley;
  if (time > 100*sec) {
    score = 45;
  } else if (time > 90*sec) {
    score = 39;
  } else if (time > 20*sec) {
    score = 24;
  } else if (time > 10*sec) {
    score = 20;
  } else {
    score = 0;
  }

  if ((time/500)%2) {
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
void displayWaitingMessage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Robot en attente...");
  display.setCursor(0, 10);
  display.println("Tirette non retiree");
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
  delay(1500);
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

void selectSequence(){
  int valeur = 0;
  if (digitalRead(SWITCH_MSB) == 1) valeur += 2;
  if (digitalRead(SWITCH_LSB) == 1) valeur += 1;

  if (movementCreated) return; // évite de reconfigurer
  movementCreated = true;

  switch(valeur){
    case 0: 
    case 2:
      movementSequence = yellowMovementSequence;
      break;
    case 1:
    case 3:
      movementSequence = blueMovementSequence;
      break;
  }

  movementSequenceNumber = 0;
  currentMovement = movementSequence[movementSequenceNumber];
}

void waitPAMIs(){
  unsigned long currentElapsed = millis() - startTime;
  if (currentElapsed >= timingPAMIs){
    currentMovement.isEnd = true;
    return;
  }
  stopMotors();
}
