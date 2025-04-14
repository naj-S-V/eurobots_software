#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>
#include "utils.h"

// ================================================================
//                           Connection
// ================================================================

#define OLED_RESET 1
#define IN1 4 
#define IN2 5 
#define IN3 6
#define IN4 8
#define CPT_US_CENTRAL_ECHO_PIN 26
#define CPT_US_CENTRAL_TRIG_PIN 27
#define CPT_US_CENTRAL_ECHO_PIN 26
#define ENC_LEFT_1 3
#define ENC_LEFT_2 2
#define ENC_RIGHT_1 19
#define ENC_RIGHT_2 18
#define RELAY 50

// ================================================================
//                    Initialisation functions
// ================================================================

void moveForward();
void turnLeft();
void turnRight180();
void turnLeft180();
void turnRight90();
void turnLeft90();

void deactivateUSSensor();
void activateUSSensor();


// ================================================================
//                           Parameters
// ================================================================

volatile float speed = 80; //Good value is 80
const float offsetRightLeft = 112.5/100; // 112.5/100

const int stopAfterSec = 10000;
const int delayStartSec = 2;
const float obstacleThreshold = 15.0;
const int ultrasonicInterval = 50;

float tickrateByDegre = 29.722;

const Movement movementSequence[] = {
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {40, moveForward, false, 0},
  {1000, turnRight90, false, 0},
  {1000, deactivateUSSensor, false, 0},
  {60, moveForward, false, 0},
};


// ================================================================
//                           Initialisation
// ================================================================

const float speedRight = speed;
const float speedLeft = speed * offsetRightLeft;

bool sensorOff = false;
int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);
float obstacleRightDistance = 0.0;
float obstacleLeftDistance = 0.0;
float obstacleCentralDistance = 0.0;
unsigned long startTime;
unsigned long lastUltrasonicCheck = 0;
bool currentSensor = false;

const int movementSequenceCount = sizeof(movementSequence) / sizeof(movementSequence[0]);
int movementSequenceNumber = 0;
Movement currentMovement = movementSequence[movementSequenceNumber];

UltrasonicSensor sensors[] = {
  {CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN, 100, false},
  {CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN, 100, false},
  {CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN, 100, false},
};
int ultrasonicSensorCount = sizeof(sensors) / sizeof(sensors[0]);

Encoder encLeft(ENC_LEFT_1, ENC_LEFT_2);
Encoder encRight(ENC_RIGHT_1, ENC_RIGHT_2);
long distanceEncLeft;
long distanceEncRight;

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

  pinMode(RELAY, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  for(int i = 0; i < ultrasonicSensorCount; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

}

// ================================================================
//                           Loop (FSM Logic)
// ================================================================
void loop() {
  unsigned long elapsedTime = (millis() - startTime)/100; // Temps écoulé en DIXIEMES de secondes
  updateScore(elapsedTime);
  readUltrasonicSensors();

  unsigned int sensorDetect = checkUltrasonicSensors();
    
  switch (currentState) {
    
    case IDLE:
      if(digitalRead(RELAY) == HIGH){
        Serial.println("tirette");
        startTime = millis();
        currentState = START;
      }
      break;
      
    case START:
      if (elapsedTime >= delayStartSec * 1000) {
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      if (sensorDetect != 1000 && !sensorOff) {
        currentState = AVOID_OBSTACLE;
      } else if (elapsedTime >= (maxTime/100)) {
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
    // Serial.println(movementSequenceNumber);
    currentMovement = movementSequence[++movementSequenceNumber];
    stopMotors();
    delay(1500);
    resetEnc();
  } else {
    currentMovement.movement();
  }
}

void deactivateUSSensor(){
  sensorOff = true;
  currentMovement.isEnd = true;
}

void activateUSSensor(){
  sensorOff = false;
  currentMovement.isEnd = true;
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

/**
 * @brief Fait avancer le robot.
 *
 * Active les moteurs pour un mouvement en avant.
 */
void moveForward() {
  // Serial.println("moveForward");
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
 * @brief Fait tourner le robot vers la droite.
 *
 * Active les moteurs de manière à effectuer une rotation vers la droite.
 */
void turnLeft() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, speedLeft);
  analogWrite(IN3, speedRight);
  digitalWrite(IN4, LOW);
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
void updateScore(int time) {
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

  if ((time/8)%2) {
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

/*
  updateUltrasonicReadings()
  Description: Updates the ultrasonic sensor readings at a fixed interval.
  Parameters: None
  Returns: None
*/
void updateUltrasonicReadings() {
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
}

