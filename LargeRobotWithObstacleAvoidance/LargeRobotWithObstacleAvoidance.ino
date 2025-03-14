/* Integration of Obstacle Detection into FSM */

#include <Wire.h>

// Inputs for motor driver
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 8
#define MOTOR_SPEED 40
#define MOVE_TIME 5000

// Ultrasonic Sensor Pins
#define CPT_US_FORWARD1_TRIG_PIN 10  // Avant gauche
#define CPT_US_FORWARD1_ECHO_PIN 11
#define CPT_US_FORWARD2_TRIG_PIN 12  // Avant droit
#define CPT_US_FORWARD2_ECHO_PIN 13
#define CPT_US_BACKWARD1_TRIG_PIN 14  // Arrière gauche
#define CPT_US_BACKWARD1_ECHO_PIN 15
#define CPT_US_BACKWARD2_TRIG_PIN 16  // Arrière droit
#define CPT_US_BACKWARD2_ECHO_PIN 17
#define CPT_US_LEFT_TRIG_PIN 18      // Côté gauche
#define CPT_US_LEFT_ECHO_PIN 19
#define CPT_US_RIGHT_TRIG_PIN 20     // Côté droit
#define CPT_US_RIGHT_ECHO_PIN 21

// Structure pour un capteur ultrason
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  int distance;
  bool isNear;
};

// Liste des capteurs
UltrasonicSensor sensors[] = {
  {CPT_US_FORWARD1_TRIG_PIN, CPT_US_FORWARD1_ECHO_PIN, 100, false},
  {CPT_US_FORWARD2_TRIG_PIN, CPT_US_FORWARD2_ECHO_PIN, 100, false},
  {CPT_US_BACKWARD1_TRIG_PIN, CPT_US_BACKWARD1_ECHO_PIN, 100, false},
  {CPT_US_BACKWARD2_TRIG_PIN, CPT_US_BACKWARD2_ECHO_PIN, 100, false},
  {CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN, 100, false},
  {CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN, 100, false}
};

// Taille du tableau
int sensorCount = sizeof(sensors) / sizeof(sensors[0]);

// FSM States
enum State {
  IDLE,
  MOVING,
  STOPPED,
  OBSTACLE
};

enum Obstacle {
  NONE = 0,
  FORWARD = 1 << 0,  // 0001
  BACKWARD = 1 << 1, // 0010
  LEFT = 1 << 2,     // 0100
  RIGHT = 1 << 3     // 1000
};


State currentState = IDLE;
unsigned long startTime = 3000;

// Obstacle Avoidance Variables
Obstacle obstacle = NONE;
unsigned long avoidance_timer = 0;

int obstacle_seuil = 15;
int obstacle_distance = 10000;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(CPT_US_OBSTACLE_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_OBSTACLE_ECHO_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  handleObstacle();  
  switch (currentState) {
    case IDLE:
      stopMotors();
      if (millis() > startTime) {
        currentState = MOVING;
      }
      break;

    case MOVING:
      moveForward();
      if (obstacle_distance <= obstacle_seuil) {
        Serial.print("Obstacle détecté à ");
        Serial.print(obstacle_distance);
        Serial.println(" cm. Début de l'évitement.");
        handleObstacleAvoidance();
        currentState = OBSTACLE;
        obstacle = FORWARD;
      }
      if (avoidance_state != 0) {
        currentState = OBSTACLE;
      }
      break;

    case OBSTACLE:
      handleObstacleAvoidance();
      if (avoidance_state == 0) {
        currentState = MOVING;
      }
      break;

    case STOPPED:
      stopMotors();
      break;
  }
}
void checkUltrason() {

  for (int i = 0; i < sensorCount; i++) {    
    digitalWrite(sensor[i].trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor[i].trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor[i].trigPin, LOW);
    long obstacle_duration = pulseIn(sensor[i].echoPin, HIGH, 3000);
    if (obstacle_duration == 0) {
      sensor[i].distance = 100;
    }
    else {
      sensor[i].distance = obstacle_duration * 0.034/2;
    }
    sensors[i].isNear = (sensors[i].distance <= 15);
  }

}


int detectObstacles() {
  checkUltrason();
  int detected = NONE;

  if (sensors[0].isNear || sensors[1].isNear) {
    detected |= FORWARD;
  }
  if (sensors[2].isNear || sensors[3].isNear) {
    detected |= BACKWARD;
  }
  if (sensors[4].isNear) {
    detected |= LEFT;
  }
  if (sensors[5].isNear) {
    detected |= RIGHT;
  }

  return detected;
}

void handleObstacles() {
  int obstacles = detectObstacles();

  int count = 0;
  if (obstacles & FORWARD) count++;
  if (obstacles & BACKWARD) count++;
  if (obstacles & LEFT) count++;
  if (obstacles & RIGHT) count++;

  if (count > 1) {
    stopMotors();
    Serial.println("STOPPING: Multiple obstacles detected.");
    return;
  }

  if (obstacles & FORWARD) {
    Serial.println("Obstacle in FRONT: Turning RIGHT until clear.");
    while (detectObstacles() & FORWARD) {
      turnRight();
      delay(100);
    }
  }
  
  else if (obstacles & BACKWARD) {
    Serial.println("Obstacle at BACK: Turning RIGHT.");
    turnRight();
  }

  else if ((obstacles & LEFT) ^ (obstacles & RIGHT)) {
    Serial.println("Obstacle on LEFT or RIGHT: Moving FORWARD.");
    moveForward();
  }
  
  else {
    Serial.println("No obstacles detected: Moving FORWARD.");
    moveForward();
  }
}


void obstacleForwardRoutine() {
  unsigned long now = millis();
  // On tourne à droite si pas d'obstacle
  checkUltrason();
  turnRight();

}
void turnRight() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, MOTOR_SPEED);
  analogWrite(IN3, MOTOR_SPEED);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, MOTOR_SPEED);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, MOTOR_SPEED);
}


void turnLeft() {
  analogWrite(IN1, MOTOR_SPEED);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, MOTOR_SPEED);
}

void moveForward() {
  analogWrite(IN1, MOTOR_SPEED);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, MOTOR_SPEED);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}