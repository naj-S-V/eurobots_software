#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Création d'une instance du shield moteur
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

#define CPT_LEFT 7         // Broche pour le capteur gauche
#define CPT_RIGHT 8        // Broche pour le capteur droit

#define CPT_US_OBSTACLE_TRIG_PIN 10
#define CPT_US_OBSTACLE_ECHO_PIN 11

#define CPT_US_TRIG_PIN 12 // Broche TRIG pour le capteur ultrason
#define CPT_US_ECHO_PIN 9 // Broche ECHO pour le capteur ultrason

volatile int speed = 60;

// Variable pour la distance mesurée
float distance = 0.0;
float obstacle_distance = 100.0;
// Seuil en cm pour la détection du vide
const float seuil = 5.0;
const float obstacle_seuil = 15.0;
// Variable booléenne pour le vide
bool isSurfaceBelow = false;

// Variable d'état pour la gestion de l'évitement
int avoidance_state = 0; // 0 = Pas d'évitement, 1 = tourner à gauche, 2 = avancer, etc.
unsigned long avoidance_timer = 0;

void setup() {
  Serial.begin(9600);
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);

  pinMode(CPT_US_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_ECHO_PIN, INPUT);
  pinMode(CPT_US_OBSTACLE_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_OBSTACLE_ECHO_PIN, INPUT);

  // Initialisation du shield moteur
  AFMS.begin();

  Serial.println("Setup terminé. Robot prêt.");
}

void loop() {
  // **Mesure de la distance avec le capteur ultrason**
  checkUltrason();

  // Priorité : arrêt immédiat si vide détecté
  if (distance >= seuil) {
    isSurfaceBelow = true;  // Vide détecté
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    Serial.println("Vide détecté ! Arrêt immédiat.");
    return; // Sortie immédiate de loop()
  } else {
    isSurfaceBelow = false; // Pas de vide
  }

  // Évitement d'obstacle
  if (obstacle_distance <= obstacle_seuil) {
    Serial.print("Obstacle détecté à ");
    Serial.print(obstacle_distance);
    Serial.println(" cm. Début de l'évitement.");
    handleObstacleAvoidance();
    return; // Sortie pour éviter d'exécuter le reste
  }

  // Si pas d'évitement actif, avancer normalement
  if (avoidance_state == 0) {
    followLine();
  }
  else {
    handleObstacleAvoidance();
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
      motorLeft->setSpeed(speed);
      motorLeft->run(BACKWARD);
      motorRight->setSpeed(speed);
      motorRight->run(FORWARD);
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
          motorLeft->setSpeed(speed);
          motorLeft->run(FORWARD);
          motorRight->setSpeed(speed);
          motorRight->run(FORWARD);
          avoidance_timer = now + 20; // Avancer par étapes courtes
        }
      }
      break;

    case 2: // Vérification d'un autre obstacle ou continuation
      if (now > avoidance_timer) {
        if (obstacle_distance <= obstacle_seuil) {
          Serial.println("Autre obstacle détecté pendant l'évitement. Recommencer.");
          avoidance_state = 0; // Recommencer l'évitement
        } else {
          Serial.println("Évitement terminé.");
          avoidance_state = 0; // Fin de l'évitement
        }
      }
      break;
  }
}

void followLine() {
  bool left = digitalRead(CPT_LEFT);   // Capteur gauche
  bool right = digitalRead(CPT_RIGHT); // Capteur droit

  Serial.print("Suivi de ligne - Capteur gauche : ");
  Serial.print(left);
  Serial.print(", Capteur droit : ");
  Serial.println(right);

  if (!left && !right) {
    // Les deux capteurs détectent du blanc -> avancer tout droit
    Serial.println("Action : avancer tout droit.");
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  } else if (!left && right) {
    // Capteur gauche détecte du blanc -> arrêt moteur gauche, moteur droit avance
    Serial.println("Action : tourner légèrement à droite.");
    motorLeft->run(RELEASE);
    motorRight->setSpeed(speed);
    motorRight->run(FORWARD);
  } else if (left && !right) {
    // Capteur droit détecte du blanc -> arrêt moteur droit, moteur gauche avance
    Serial.println("Action : tourner légèrement à gauche.");
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
  } else {
    // Les deux capteurs détectent du noir -> avancer tout droit
    Serial.println("Action : avancer tout droit (zone noire).");
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
}
