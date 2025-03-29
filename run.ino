#include <Servo.h>

#define MOTOR_FORWARD_PIN 4
#define MOTOR_BACKWARD_PIN 5
#define RELAY_PIN 9
#define SERVO_PIN 3

Servo myServo;

void setup() {
    pinMode(MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_BACKWARD_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    myServo.attach(SERVO_PIN);
}

void loop() {
    // Étape 0 : Activer le moteur DC (aller vers l'avant)
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    delay(1000);
    
    // Étape 1 : Reculer pendant 3 secondes
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
    delay(3000);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    
    // Étape 2 : Avancer pendant 0.7 seconde
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    delay(700);
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    
    // Étape 3 : Ouvrir complètement le servo
    myServo.write(180);
    delay(500);
    
    // Étape 4 : Activer le relais pendant 2 secondes
    digitalWrite(RELAY_PIN, HIGH);
    delay(2000);
    
    // Étape 5 : Désactiver le relais
    digitalWrite(RELAY_PIN, LOW);
    
    // Étape 6 : Aller dans l'autre sens (donner -5V)
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
    delay(1000);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    
    // Étape 7 : Alterner entre avant et arrière pendant 4 secondes
    for (int i = 0; i < 10; i++) {
        digitalWrite(MOTOR_FORWARD_PIN, HIGH);
        delay(200);
        digitalWrite(MOTOR_FORWARD_PIN, LOW);
        
        digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
        delay(200);
        digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    }
}
