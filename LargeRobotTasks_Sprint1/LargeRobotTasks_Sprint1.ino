#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 8
#define MOTOR_SPEED 150
#define MOTOR_SPEED_SLOW (MOTOR_SPEED)
#define MOVE_TIME 5000

enum State {
  IDLE,
  MOVING,
  STOPPED
};

State currentState = IDLE;
unsigned long startTime = 3000;

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
  analogWrite(IN1, MOTOR_SPEED);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, MOTOR_SPEED_SLOW);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
