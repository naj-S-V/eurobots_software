#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Création d'une instance du shield moteur
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
#define ENC_LEFT 3   // Broche de l'encodeur du moteur gauche
#define ENC_RIGHT 2  // Broche de l'encodeur du moteur droit

#define CPT_LEFT 7
#define CPT_RIGHT 8

#define CPT_US_TRIG_PIN 10
#define CPT_US_ECHO_PIN 11

#define SPEED 50
volatile int counter = 0;
bool estProche = false;
// Variable pour la distance
float US_DIST = 100.0;
// Distance seuil en cm pour détecter le vide
const float seuil = 10.0;
// Variable booléenne pour le vide
bool isSurfaceBelow = true;
#define TICKS_PER_CM 1  // 330/(pi*5.8)
#define DISTANCE_CM 1000  // Distance cible en cm

// Variables pour compter les ticks
volatile int ticksLeft = 0;
volatile int ticksRight = 0;
volatile unsigned long lastTickTimeLeft = 0;
volatile unsigned long lastTickTimeRight = 0;
const unsigned long debounceDelay = 5; // En millisecondes

void countLeft() {
  unsigned long currentTime = millis();
  if (currentTime - lastTickTimeLeft > debounceDelay) {
    ticksLeft++;
    lastTickTimeLeft = currentTime;
    
  }
}

void countRight() {
  unsigned long currentTime = millis();
  if (currentTime - lastTickTimeRight > debounceDelay) {
    ticksRight++;
    lastTickTimeRight = currentTime;
  }
}



void setup() {
  // Initialisation du shield moteur
  AFMS.begin();
  // Démarrer la communication série
  Serial.begin(9600);
  // while (!Serial); // Attendre que la connexion série soit établie

  // // Log que le programme a démarré
  // Serial.println("Programme démarré!");

  pinMode(CPT_LEFT, INPUT);
  pinMode(CPT_RIGHT, INPUT);

    // Configuration des broches d'encodeur
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  // Attachement des interruptions pour les encodeurs
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), countRight, RISING);

  pinMode(CPT_US_TRIG_PIN, OUTPUT);
  pinMode(CPT_US_ECHO_PIN, INPUT);

  //delay(5000);  // Attendre l'initialisation du shield moteur

  // Log que le shield moteur est initialisé
  // Serial.println("Shield moteur initialisé.");
}

void randomSpin() {
  int direction = random(0, 2);  // 0 pour gauche, 1 pour droite
  int spinTime = random(500, 2000);  // Durée aléatoire entre 500ms et 2000ms

  Serial.print("Spinning ");
  if (direction == 0) {
    Serial.println("LEFT");
    myMotor1->setSpeed(SPEED);
    myMotor2->setSpeed(SPEED);
    myMotor1->run(BACKWARD);
    myMotor2->run(FORWARD);
  } else {
    Serial.println("RIGHT");
    myMotor1->setSpeed(SPEED);
    myMotor2->setSpeed(SPEED);
    myMotor1->run(FORWARD);
    myMotor2->run(BACKWARD);
  }

  delay(spinTime);  // Maintient la rotation pour la durée aléatoire
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}
void loop() {
  // Lire les valeurs des capteurs infrarouges
  bool left = digitalRead(CPT_LEFT);  // Détection du noir
  bool right = digitalRead(CPT_RIGHT); // Détection du noir
    // Calcul du nombre total de ticks requis pour atteindre la distance cible
  int targetTicks = DISTANCE_CM * TICKS_PER_CM;
  // Log des valeurs détectées par les capteurs infrarouges
  // Serial.print("CPT_LEFT: ");
  // Serial.print(left);
  // Serial.print("  CPT_RIGHT: ");
  // Serial.println(right);
  // Affiche les ticks actuels (pour le débogage)
  // Génération de l'impulsion sur le Trig du capteur ultrason
  counter = counter + 1;
  Serial.print(counter);
  //si counter un multiple de 10
  if (counter % 10 == 0) {
    checkUltrason();
  }
  digitalWrite(CPT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_TRIG_PIN, LOW);

  // Lecture de l'impulsion sur l'Echo
  long duration = pulseIn(CPT_US_ECHO_PIN, HIGH, 30000);

  // Calcul de la distance en cm
  US_DIST = (duration * 0.034) / 2;
  // Mise à jour de la variable booléenne
  if (US_DIST > seuil) {
    estProche = false;
  } else {
    estProche = true;
  }
  Serial.print(US_DIST);
  // Vérification du vide
  // if (isSurfaceBelow) {
  //   Serial.println("VIDE DÉTECTÉ : STOP");
  //   myMotor1->run(RELEASE);
  //   myMotor2->run(RELEASE);
  //   return; // Fin immédiate de la boucle pour éviter toute autre action
  // }

  // Contrôle des moteurs en fonction des capteurs infrarouges
    if (!estProche){
      if (left && right) {
        //Serial.println("FORWARD");
        myMotor1->setSpeed(SPEED);
        myMotor2->setSpeed(SPEED);
        myMotor1->run(FORWARD);
        myMotor2->run(FORWARD);
        delay(20);
        myMotor2->run(RELEASE);
        myMotor1->run(RELEASE);
      } else if (!left && right) {
        //Serial.println("LEFT");
        myMotor2->setSpeed(SPEED);
        myMotor1->run(RELEASE);
        myMotor2->run(FORWARD);
        delay(20);
        myMotor2->run(RELEASE);
      } else if (left && !right) {
        //Serial.println("RIGHT");
        myMotor1->setSpeed(SPEED);
        myMotor1->run(FORWARD);
        myMotor2->run(RELEASE);
        delay(20); 
        myMotor1->run(RELEASE);

    // } else {
    //   //Serial.println("STOP");
    //   myMotor1->run(FORWARD);
    //   myMotor2->run(FORWARD);
    
    //   }

    }
    else if (!left && !right) {
      //Serial.println("RIGHT");
      myMotor2->run(RELEASE);
      myMotor1->run(RELEASE);

    
    
    if (!(ticksLeft >= targetTicks && ticksRight >= targetTicks)) {
        myMotor1->setSpeed(SPEED);
        myMotor2->setSpeed(SPEED);
        myMotor1->run(FORWARD);
        myMotor2->run(FORWARD); 
        delay(20);
        myMotor1->run(RELEASE);
        myMotor2->run(RELEASE);
        randomSpin(); 
    }
  }
}

else {
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}
}

void checkUltrason() {
  
  digitalWrite(CPT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CPT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CPT_US_TRIG_PIN, LOW);

  // Lecture de l'impulsion sur l'Echo
  long duration = pulseIn(CPT_US_ECHO_PIN, HIGH, 10000);

  // Calcul de la distance en cm
  US_DIST = (duration * 0.034) / 2;
  // Mise à jour de la variable booléenne
  if (US_DIST > seuil) {
    estProche = false;
  } else {
    estProche = true;
  }
  Serial.print(US_DIST);
}
  // Pas de delay pour assurer une boucle continue
