#include <Wire.h>

// ================================================================
//                           Connection
// ================================================================

#define IN1 4 
#define IN2 5 
#define IN3 6
#define IN4 8
#define CPT_US_RIGHT_TRIG_PIN 35
#define CPT_US_RIGHT_ECHO_PIN 34
#define CPT_US_LEFT_TRIG_PIN 31
#define CPT_US_LEFT_ECHO_PIN 30
#define CPT_US_CENTRAL_TRIG_PIN 27
#define CPT_US_CENTRAL_ECHO_PIN 26

// ================================================================
//                           Parameters
// ================================================================

volatile int speed = 60;

const int stopAfterSec = 10000;
const int delayStartSec = 5;
const float obstacleThreshold = 30.0;
const int ultrasonicInterval = 50;

// ================================================================
//                         Struct and enum
// ================================================================

/**
 * @brief Structure pour un capteur ultrason.
 *
 * Contient les informations de pin, la distance mesurée et un booléen indiquant si un obstacle est
 * détecté à proximité.
 */
struct UltrasonicSensor {
  int trigPin;    ///< Pin de déclenchement
  int echoPin;    ///< Pin d'écho
  int distance;   ///< Distance mesurée en cm
  bool isNear;    ///< Indique si un obstacle est à moins de 15 cm
};

// ================================================================
//                           Initialisation
// ================================================================

int maxTime = (stopAfterSec * 1000) + (delayStartSec * 1000);
float obstacleRightDistance = 0.0;
float obstacleLeftDistance = 0.0;
float obstacleCentralDistance = 0.0;
unsigned long startTime;
unsigned long lastUltrasonicCheck = 0;
bool currentSensor = false;

UltrasonicSensor sensors[] = {
  {CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN, 100, false},
  {CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN, 100, false},
  {CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN, 100, false},
};
int ultrasonicSensorCount = sizeof(sensors) / sizeof(sensors[0]);

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

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  for(int i = 0; i < ultrasonicSensorCount; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }
}

// ================================================================
//                           Loop (FSM Logic)
// ================================================================
void loop() {
  unsigned long elapsedTime = millis() - startTime;
    
  readUltrasonicSensors();

  unsigned int sensorDetect = checkUltrasonicSensors();
  // Serial.println(sensorDetect);
  
  switch (currentState) {
    
    case IDLE:
      if (elapsedTime >= delayStartSec * 1000) {
        currentState = RUNNING;
      }
      break;
    
    case RUNNING:
      if (sensorDetect != 1000) {
        currentState = AVOID_OBSTACLE;
      } else if (elapsedTime >= maxTime) {
        currentState = STOPPED;
      } else {
        moveForward();      
      }
      break;
    
    case AVOID_OBSTACLE:
      avoidObstacle(sensorDetect);
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

void avoidObstacle(int sensor){
  if (sensor == 0) { 
    avoidObstacleLeft();
  } else {
    avoidObstacleRight();
  }
}

void avoidObstacleRight(){
  turnLeft();
  delay(500);
}

void avoidObstacleLeft(){
  turnRight();
  delay(500);
}

/**
 * @brief Fait avancer le robot.
 *
 * Active les moteurs pour un mouvement en avant.
 */
void moveForward() {
  analogWrite(IN1, speed);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, speed);
  digitalWrite(IN4, LOW);
}

/**
 * @brief Arrête tous les moteurs du robot.
 *
 * Met les pins des moteurs à LOW pour arrêter tout mouvement.
 */
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
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

/**
 * @brief Fait tourner le robot vers la gauche.
 *
 * Active les moteurs pour réaliser une rotation vers la gauche.
 */
void turnLeft() {
  analogWrite(IN1, speed);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, speed);
}

/**
 * @brief Fait tourner le robot vers la droite.
 *
 * Active les moteurs de manière à effectuer une rotation vers la droite.
 */
void turnRight() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, speed);
  analogWrite(IN3, speed);
  digitalWrite(IN4, LOW);
}

