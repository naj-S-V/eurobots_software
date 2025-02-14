#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Création d'une instance du shield moteur
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

#define CPT_LEFT 7         // Broche pour le capteur gauche
#define CPT_RIGHT 8     // Broche pour le capteur droit

#define CPT_US_TRIG_PIN 12 // Broche TRIG pour le capteur ultrason
#define CPT_US_ECHO_PIN 13 // Broche ECHO pour le capteur ultrason

volatile int speed = 60;

// Variable pour la distance mesurée
float distance = 0.0;
// Seuil en cm pour la détection du vide
const float seuil = 5.0;
// Variable booléenne pour le vide
bool isSurfaceBelow = false;

volatile int counter = 1;
const int max_int = 0;



void setup() {
  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);

  pinMode(CPT_US_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_ECHO_PIN, INPUT);

  // Initialisation du shield moteur
  AFMS.begin();

  // Initialisation du moniteur série pour le débogage
  //Serial.begin(9600);
}

void loop() {
  // **Mesure de la distance avec le capteur ultrason**
  digitalWrite(CPT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_TRIG_PIN, LOW);

  long duration = pulseIn(CPT_US_ECHO_PIN, HIGH,10000);
  distance = (duration * 0.034) / 2; // Conversion en cm

  // Debug : Afficher la distance mesurée
  //Serial.print("Distance mesurée : ");
  //Serial.println(distance);

  counter = counter + 1;
  if (counter >= 600){
    speed = 40;
  }

  // **Priorité : arrêt immédiat si vide détecté**
  if (distance >= seuil) {
    isSurfaceBelow = true;  // Vide détecté
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);

    //Serial.println("Vide détecté ! Arrêt immédiat des moteurs.");
    //delay(100); // Petite pause pour éviter les lectures trop rapides
    return;     // Sortie immédiate de loop()
  } else {
    isSurfaceBelow = false; // Pas de vide
  }

  // **Lecture des capteurs infrarouges**
  bool left = digitalRead(CPT_LEFT);   // Capteur gauche
  bool right = digitalRead(CPT_RIGHT); // Capteur droit

  // **Gestion des capteurs infrarouges uniquement si pas de vide**
  if (!left && !right) {
    // Les deux capteurs détectent du blanc -> avancer tout droit
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  } else if (!left && right) {
    // Capteur gauche détecte du blanc -> arrêt moteur gauche, moteur droit avance
    motorLeft->run(RELEASE);
    motorRight->setSpeed(speed);
    motorRight->run(FORWARD);
  } else if (left && !right) {
    // Capteur droit détecte du blanc -> arrêt moteur droit, moteur gauche avance
    motorRight->run(RELEASE);
    motorLeft->setSpeed(speed);
    motorLeft->run(FORWARD);
  } else {
    // Les deux capteurs détectent du noir -> avancer tout droit
    motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
  Serial.println(speed);
  delay(10);
}
