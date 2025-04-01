/**
 * @file ObstacleDetectionFSM.ino
 * @brief Contrôle d'un robot avec détection d'obstacles intégrée dans une FSM (Machine à États Finis)
 *
 * Ce programme utilise des capteurs ultrason pour détecter des obstacles sur plusieurs directions
 * (avant, arrière, gauche et droite) et ajuste le comportement du robot en fonction d'une machine
 * à états (IDLE, MOVING, OBSTACLE, STOPPED). Les actions du robot incluent avancer, tourner et s'arrêter.
 *
 * Certaines fonctions (ex. handleObstacleAvoidance) et variables (ex. avoidance_state) ne sont pas définies
 * dans ce snippet et devront être implémentées selon vos besoins.
 *
 * @author  
 * @date    
 */

#include <Wire.h>

// Définitions des pins pour le moteur
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 8
#define MOTOR_SPEED 40
#define MOVE_TIME 5000

// Définitions des pins pour les capteurs ultrason
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

// ATTENTION : Les pins pour un capteur d'obstacle unique sont utilisés dans setup()
// mais ne sont pas définies dans ce snippet. Pensez à les définir si nécessaire.
#define CPT_US_OBSTACLE_TRIG_PIN 22
#define CPT_US_OBSTACLE_ECHO_PIN 23

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

// Liste des capteurs utilisés pour la détection dans plusieurs directions.
UltrasonicSensor sensors[] = {
  {CPT_US_FORWARD1_TRIG_PIN, CPT_US_FORWARD1_ECHO_PIN, 100, false},
  {CPT_US_FORWARD2_TRIG_PIN, CPT_US_FORWARD2_ECHO_PIN, 100, false},
  {CPT_US_BACKWARD1_TRIG_PIN, CPT_US_BACKWARD1_ECHO_PIN, 100, false},
  {CPT_US_BACKWARD2_TRIG_PIN, CPT_US_BACKWARD2_ECHO_PIN, 100, false},
  {CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN, 100, false},
  {CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN, 100, false}
};

/// Nombre de capteurs disponibles
int sensorCount = sizeof(sensors) / sizeof(sensors[0]);

// Définition des états de la FSM
enum State {
  IDLE,     ///< Le robot est à l'arrêt et attend le début
  MOVING,   ///< Le robot est en mouvement
  STOPPED,  ///< Le robot est arrêté (peut être utilisé pour une condition d'urgence)
  OBSTACLE  ///< Le robot est en mode évitement d'obstacle
};

// Définition des obstacles par direction avec un masque binaire
enum Obstacle {
  NONE = 0,
  FORWARD = 1 << 0,  ///< Obstacle à l'avant
  BACKWARD = 1 << 1, ///< Obstacle à l'arrière
  LEFT = 1 << 2,     ///< Obstacle sur la gauche
  RIGHT = 1 << 3     ///< Obstacle sur la droite
};

State currentState = IDLE;   ///< État initial de la machine à états
unsigned long startTime = 3000;  ///< Temps d'attente avant de démarrer (en ms)

// Variables pour la détection et l'évitement d'obstacles
Obstacle obstacle = NONE;
unsigned long avoidance_timer = 0;
int obstacle_seuil = 15;         ///< Seuil de distance (cm) pour considérer un obstacle proche
int obstacle_distance = 10000;   ///< Distance d'obstacle (valeur initiale élevée)

/**
 * @brief Fonction d'initialisation.
 *
 * Configure les pins des moteurs et des capteurs ultrason, et initialise la communication série.
 */
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(CPT_US_OBSTACLE_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_OBSTACLE_ECHO_PIN, INPUT);
  Serial.begin(9600);
}

/**
 * @brief Boucle principale de la FSM.
 *
 * Gère les différents états (IDLE, MOVING, OBSTACLE, STOPPED) et appelle les fonctions
 * associées pour le mouvement et l'évitement d'obstacles.
 */
void loop() {
  // Traitement des obstacles (vérification et gestion)
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
        handleObstacleAvoidance(); // Fonction à implémenter
        currentState = OBSTACLE;
        obstacle = FORWARD;
      }
      // La variable avoidance_state n'est pas définie dans ce snippet
      if (avoidance_state != 0) {
        currentState = OBSTACLE;
      }
      break;

    case OBSTACLE:
      handleObstacleAvoidance(); // Fonction à implémenter
      if (avoidance_state == 0) {
        currentState = MOVING;
      }
      break;

    case STOPPED:
      stopMotors();
      break;
  }
}

/**
 * @brief Mesure la distance avec tous les capteurs ultrason.
 *
 * Pour chaque capteur, envoie une impulsion et mesure la durée de l'écho afin de calculer la distance.
 * Met à jour le champ distance et le booléen isNear pour chaque capteur.
 */
void checkUltrason() {
  for (int i = 0; i < sensorCount; i++) {    
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
    sensors[i].isNear = (sensors[i].distance <= 15);
  }
}

/**
 * @brief Détecte les obstacles à partir des capteurs ultrason.
 *
 * Utilise la fonction checkUltrason() pour mettre à jour les mesures, puis retourne
 * un masque binaire indiquant la présence d'obstacles dans chaque direction.
 *
 * @return int Masque binaire combinant FORWARD, BACKWARD, LEFT, RIGHT si un obstacle est détecté.
 */
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

/**
 * @brief Gère la réaction du robot face aux obstacles détectés.
 *
 * Selon le nombre et la position des obstacles, le robot arrête ses moteurs ou effectue
 * des manœuvres (tourner ou avancer). Pour plusieurs obstacles, il s'arrête complètement.
 */
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

/**
 * @brief Routine d'évitement pour un obstacle à l'avant.
 *
 * Si aucun obstacle n'est détecté sur le côté droit, le robot tourne vers la droite.
 * La fonction utilise checkUltrason() pour mettre à jour les mesures avant de tourner.
 */
void obstacleForwardRoutine() {
  unsigned long now = millis();
  checkUltrason();
  turnRight();
}

/**
 * @brief Fait tourner le robot vers la droite.
 *
 * Active les moteurs de manière à effectuer une rotation vers la droite.
 */
void turnRight() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, MOTOR_SPEED);
  analogWrite(IN3, MOTOR_SPEED);
  digitalWrite(IN4, LOW);
}

/**
 * @brief Fait reculer le robot.
 *
 * Active les moteurs pour effectuer un mouvement en marche arrière.
 */
void moveBackward() {
  digitalWrite(IN1, LOW);
  analogWrite(IN2, MOTOR_SPEED);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, MOTOR_SPEED);
}

/**
 * @brief Fait tourner le robot vers la gauche.
 *
 * Active les moteurs pour réaliser une rotation vers la gauche.
 */
void turnLeft() {
  analogWrite(IN1, MOTOR_SPEED);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  analogWrite(IN4, MOTOR_SPEED);
}

/**
 * @brief Fait avancer le robot.
 *
 * Active les moteurs pour un mouvement en avant.
 */
void moveForward() {
  analogWrite(IN1, MOTOR_SPEED);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, MOTOR_SPEED);
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
