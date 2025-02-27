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
#define CPT_US_OBSTACLE_TRIG_PIN 10
#define CPT_US_OBSTACLE_ECHO_PIN 11

// FSM States
enum State {
  IDLE,
  MOVING,
  STOPPED,
  OBSTACLE
};

State currentState = IDLE;
unsigned long startTime = 3000;

// Obstacle Avoidance Variables
int avoidance_state = 0; // 0 = No avoidance, 1 = Turn left, 2 = Forward
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
  checkUltrason();
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
  digitalWrite(CPT_US_OBSTACLE_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_OBSTACLE_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_OBSTACLE_TRIG_PIN, LOW);

  long obstacle_duration = pulseIn(CPT_US_OBSTACLE_ECHO_PIN, HIGH, 30000);

  if (obstacle_duration == 0) {
    obstacle_distance = 10000.0; // Réinitialiser si aucune mesure
  } else {
    obstacle_distance = (obstacle_duration * 0.034) / 2; // Conversion en cm
  }

  Serial.print("Distance à l'obstacle : ");
  Serial.print(obstacle_distance);
  Serial.println(" cm");
  Serial.println(avoidance_timer);
  Serial.println(avoidance_state);
}

void handleObstacleAvoidance() {
  unsigned long now = millis();

  switch (avoidance_state) {
    case 0: // Début de l'évitement, tourner à gauche
      Serial.println("Évitement étape 1 : tourner à gauche.");
      turnRight();
      avoidance_state = 1;
      avoidance_timer = now + 500; // Durée pour tourner
      break;

    case 1: // Avancer tout droit (en vérifiant les obstacles)
      if (now > avoidance_timer) {
        if (obstacle_distance <= obstacle_seuil) {
          Serial.println("Obstacle toujours présent. Recommencer l'évitement.");
          avoidance_state = 0; // Recommencer l'évitement
        } else {
          Serial.println("Évitement étape 2 : avancer tout droit.");
          currentState = MOVING;
        }
      }
      break;
  }
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