#include <Servo.h>

#define MOTOR_IN1 4
#define MOTOR_IN2 5
#define MOTOR_IN3 6
#define MOTOR_IN4 8
#define RELAY_PIN 9
#define SERVO_PIN 3
#define MOTOR_SPEED 255

Servo servoMotor;

void setup() {
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    servoMotor.attach(SERVO_PIN);
}

void loop() {
    // Étape 0: Activer le moteur DC avec 5V
    analogWrite(MOTOR_IN1, MOTOR_SPEED);
    digitalWrite(MOTOR_IN2, LOW);
    delay(1000);

    // Étape 1: Reculer pendant 3 secondes
    analogWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, MOTOR_SPEED);
    analogWrite(MOTOR_IN3, LOW);
    analogWrite(MOTOR_IN4, MOTOR_SPEED);
    delay(3000);

    // Étape 2: Avancer pendant 0.7 secondes
    analogWrite(MOTOR_IN1, MOTOR_SPEED);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN3, MOTOR_SPEED);
    digitalWrite(MOTOR_IN4, LOW);
    delay(700);

    // Étape 3: Ouvrir complètement le servo
    servoMotor.write(180);
    delay(1000);

    // Étape 4: Activer le relais pendant 2 secondes
    digitalWrite(RELAY_PIN, HIGH);
    delay(2000);

    // Étape 5: Désactiver le relais
    digitalWrite(RELAY_PIN, LOW);

    // Étape 6: Donner -5V au moteur DC (inversion de polarité)
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, MOTOR_SPEED);
    delay(1000);

    // Étape 7: Alterner marche avant/arrière pendant 4 secondes (0.2s chaque cycle)
    for (int i = 0; i < 20; i++) {
        analogWrite(MOTOR_IN1, MOTOR_SPEED);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_IN3, MOTOR_SPEED);
        digitalWrite(MOTOR_IN4, LOW);
        delay(200);

        analogWrite(MOTOR_IN1, LOW);
        analogWrite(MOTOR_IN2, MOTOR_SPEED);
        analogWrite(MOTOR_IN3, LOW);
        analogWrite(MOTOR_IN4, MOTOR_SPEED);
        delay(200);
    }
}
