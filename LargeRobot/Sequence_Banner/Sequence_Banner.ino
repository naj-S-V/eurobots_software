#include <Servo.h>

#define PIN_PINCE_FERMETURE 48
#define PIN_VENTILLO 47
#define PIN_PINCE_OUVERTURE 46
#define PIN_SERVO 49
#define PIN_EJECTEUR 50
#define PIN_EJECTEUR_RETRACTION 51

// Définition des constantes de temps
#define TIME_1 3000
#define TIME_2 4000
#define TIME_3 1500
#define TIME_4 4000
#define TIME_5 4000
#define TIME_6 1000
#define TIME_7 2200

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
    pinMode(PIN_EJECTEUR, OUTPUT);
    pinMode(PIN_EJECTEUR_RETRACTION, OUTPUT);
    myServo.attach(PIN_SERVO);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    myServo.write(10);
    digitalWrite(PIN_EJECTEUR_RETRACTION, HIGH);
    delay(TIME_7); // Temps pour éjecter
    digitalWrite(PIN_EJECTEUR_RETRACTION, LOW);
    
    // Étape 1 : Activation fermeture pince
    digitalWrite(PIN_PINCE_FERMETURE, HIGH);
    delay(TIME_1);
    digitalWrite(PIN_PINCE_FERMETURE, LOW);

    // Étape 2 : Déplacement du servo
    myServo.write(120);
    delay(1000); // Temps pour stabilisation
    
    // Étape 3 : Avancer
    moveForward();
    delay(TIME_3);
    stopMotors();

    // Étape 4 : Activation ventilo
    digitalWrite(PIN_VENTILLO, HIGH);
    delay(TIME_4);
    
    // Étape 5 : Reculer
    moveBackward();
    delay(TIME_2);
    stopMotors();

    // Étape intermédiaire : Avancer très légèrement
    moveForward();
    delay(TIME_6);
    stopMotors();

    // Étape 6 : Éteindre ventillo
    digitalWrite(PIN_VENTILLO, LOW);

    // Étape 7 : Ouverture pince
    digitalWrite(PIN_PINCE_OUVERTURE, HIGH);
    delay(TIME_5);
    digitalWrite(PIN_PINCE_OUVERTURE, LOW);

    // Étape 8 : Activer éjecteur
    digitalWrite(PIN_EJECTEUR, HIGH);
    delay(TIME_7); // Temps pour éjecter
    digitalWrite(PIN_EJECTEUR, LOW);

    // RESET SECTION

    myServo.write(10);
    digitalWrite(PIN_EJECTEUR_RETRACTION, HIGH);
    delay(TIME_7); // Temps pour éjecter
    digitalWrite(PIN_EJECTEUR_RETRACTION, LOW);
    
    

    
    // Fin de la séquence (ne pas répéter)
    while (true);
}

void moveForward() {
    analogWrite(IN1, MOTOR_SPEED*1.12);
    digitalWrite(IN2, LOW);
    analogWrite(IN3, MOTOR_SPEED);
    digitalWrite(IN4, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, MOTOR_SPEED*1.12);
    digitalWrite(IN3, LOW);
    analogWrite(IN4, MOTOR_SPEED);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
