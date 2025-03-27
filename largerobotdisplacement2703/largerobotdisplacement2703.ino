#include <Servo.h>  // Inclusion de la librairie Servo

// Définitions des pins pour le moteur
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 8
#define MOTOR_SPEED 40

// Définitions des pins pour le capteur ultrason à l'avant
#define TRIG_PIN 10
#define ECHO_PIN 11

// Définition de la pin pour le servo moteur
#define SERVO_PIN 3

// Seuil de détection d'obstacle en cm
#define OBSTACLE_THRESHOLD 25

// Délai entre chaque mesure (en millisecondes)
#define DELAY_TIME 200

Servo myServo;  // Création de l'objet servo

/**
 * @brief Initialisation.
 *
 * Configure les pins pour les moteurs, le capteur ultrason et attache le servo moteur.
 */
void setup() {
  // Configuration des pins moteurs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuration du capteur ultrason
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Attache le servo à la pin définie et initialise sa position
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Position par défaut

  // Initialisation de la communication série pour afficher les mesures
  Serial.begin(9600);
}

/**
 * @brief Boucle principale.
 *
 * Mesure la distance à l'avant et commande le robot :
 * - Avancer si aucun obstacle n'est détecté (distance > OBSTACLE_THRESHOLD) et le servo reste à 0°.
 * - S'arrêter et activer le servo (le déplacer à 90°) dès qu'un obstacle est détecté.
 */
void loop() {
  int distance = measureDistance();

  Serial.print("Distance avant : ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > OBSTACLE_THRESHOLD) {
    moveForward();
    // Servo en position par défaut
    myServo.write(0);
  } else {
    stopMotors();
    Serial.println("Obstacle détecté ! Arrêt et activation du servo.");
    // Déplacement du servo à 90° en cas d'obstacle détecté
    myServo.write(random(20,180));
  }

  delay(DELAY_TIME);
}

/**
 * @brief Mesure la distance avec le capteur ultrason.
 *
 * @return int Distance mesurée en cm.
 */
int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 3000);
  if (duration == 0) return 1000;  // Si aucune impulsion, on retourne une grande distance
  return duration * 0.034 / 2;
}

/**
 * @brief Fait avancer le robot.
 */
void moveForward() {
  analogWrite(IN1, MOTOR_SPEED);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, MOTOR_SPEED);
  digitalWrite(IN4, LOW);
}

/**
 * @brief Arrête tous les moteurs du robot.
 */
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
