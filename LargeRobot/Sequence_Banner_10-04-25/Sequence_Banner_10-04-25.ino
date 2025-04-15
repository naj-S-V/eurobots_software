#include <Servo.h>

#define PIN_PINCE_FERMETURE 48
#define PIN_VENTILLO 47
#define PIN_PINCE_OUVERTURE 46
#define PIN_SERVO 49

// Définition des constantes de temps
#define TIME_1 2500
#define TIME_2 4000
#define TIME_3 1500
#define TIME_4 7000
#define TIME_5 1500
#define TIME_6 200

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
    digitalWrite(PIN_PINCE_FERMETURE, HIGH);
    delay(TIME_1);
    digitalWrite(PIN_PINCE_FERMETURE, LOW);
}

void loop() {
    // Étape 1 : Activation fermeture pince

    digitalWrite(PIN_VENTILLO, HIGH);
    delay(TIME_4);
    

    // Étape 6 : Éteindre ventillo
    digitalWrite(PIN_VENTILLO, LOW);
    delay(TIME_5);


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
