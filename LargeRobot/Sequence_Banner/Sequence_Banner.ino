#include <Servo.h>

#define PIN_PINCE_FERMETURE 46
#define PIN_VENTILLO 47
#define PIN_PINCE_OUVERTURE 48
#define PIN_SERVO 49

// Définition des constantes de temps
#define TIME_1 1000
#define TIME_2 2000
#define TIME_3 3000
#define TIME_4 4000
#define TIME_5 5000

// Définitions des pins pour les moteurs
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 8
#define MOTOR_SPEED 40

Servo myServo;

void setup() {
    pinMode(PIN_PINCE_FERMETURE, OUTPUT);
    pinMode(PIN_VENTILLO, OUTPUT);
    pinMode(PIN_PINCE_OUVERTURE, OUTPUT);
    myServo.attach(PIN_SERVO);
    myServo.write(0);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    // Étape 1 : Activation fermeture pince
    digitalWrite(PIN_PINCE_FERMETURE, HIGH);
    delay(TIME_1);
    digitalWrite(PIN_PINCE_FERMETURE, LOW);

    // Étape 2 : Reculer
    moveBackward();
    delay(TIME_2);
    stopMotors();

    // Étape 3 : Avancer
    moveForward();
    delay(TIME_3);
    stopMotors();

    // Étape 4 : Déplacement du servo
    myServo.write(120);
    delay(1000); // Temps pour stabilisation

    // Étape 5 : Activation ventilo
    digitalWrite(PIN_VENTILLO, HIGH);
    delay(TIME_4);
    digitalWrite(PIN_VENTILLO, LOW);

    // Étape 6 : Ouverture pince
    digitalWrite(PIN_PINCE_OUVERTURE, HIGH);
    delay(TIME_5);
    digitalWrite(PIN_PINCE_OUVERTURE, LOW);

    // Fin de la séquence (ne pas répéter)
    while (true);
}

void moveForward() {
    analogWrite(IN1, MOTOR_SPEED);
    digitalWrite(IN2, LOW);
    analogWrite(IN3, MOTOR_SPEED);
    digitalWrite(IN4, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, MOTOR_SPEED);
    digitalWrite(IN3, LOW);
    analogWrite(IN4, MOTOR_SPEED);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
