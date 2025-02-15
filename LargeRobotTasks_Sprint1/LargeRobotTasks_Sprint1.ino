// Configuration hardware : Les broches IN du driver moteur sont connectées aux pins du Arduino Mega de 4 à 7
// To Do : Ajouter les encodeurs dans le code

#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define MOTOR_SPEED 150
#define MOVE_TIME 2000

enum State {
  IDLE,
  MOVING,
  STOPPED
};

State currentState = IDLE;
unsigned long startTime = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.begin(9600);
  currentState = MOVING;
}

void loop() {
  switch (currentState) {
    case IDLE:
      break;

    case MOVING:
      Serial.println("Le robot avance...");
      moveForward();
      startTime = millis();
      currentState = STOPPED;
      break;

    case STOPPED:
      if (millis() - startTime >= MOVE_TIME) {
        stopMotors();
        Serial.println("Le robot s'arrête.");
        currentState = IDLE;
      }
      break;
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
