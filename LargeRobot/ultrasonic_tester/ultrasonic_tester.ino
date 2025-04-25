// === Définition des pins ===
#define CPT_US_RIGHT_TRIG_PIN 35
#define CPT_US_RIGHT_ECHO_PIN 34 
#define CPT_US_LEFT_TRIG_PIN 31
#define CPT_US_LEFT_ECHO_PIN 30
#define CPT_US_CENTRAL_TRIG_PIN 27
#define CPT_US_CENTRAL_ECHO_PIN 26
#define CPT_US_BACK_RIGHT_TRIG_PIN 39
#define CPT_US_BACK_RIGHT_ECHO_PIN 38
#define CPT_US_BACK_LEFT_TRIG_PIN 41
#define CPT_US_BACK_LEFT_ECHO_PIN 40
#define CPT_US_BACK_CENTRAL_TRIG_PIN 28
#define CPT_US_BACK_CENTRAL_ECHO_PIN 29

struct UltraPair {
  const char* name;
  int trig;
  int echo;
};

UltraPair sensors[] = {
  {"FRONT RIGHT", CPT_US_RIGHT_TRIG_PIN, CPT_US_RIGHT_ECHO_PIN},
  {"FRONT LEFT", CPT_US_LEFT_TRIG_PIN, CPT_US_LEFT_ECHO_PIN},
  {"FRONT CENTER", CPT_US_CENTRAL_TRIG_PIN, CPT_US_CENTRAL_ECHO_PIN},
  {"BACK RIGHT", CPT_US_BACK_RIGHT_TRIG_PIN, CPT_US_BACK_RIGHT_ECHO_PIN},
  {"BACK LEFT", CPT_US_BACK_LEFT_TRIG_PIN, CPT_US_BACK_LEFT_ECHO_PIN},
  {"BACK CENTER", CPT_US_BACK_CENTRAL_TRIG_PIN, CPT_US_BACK_CENTRAL_ECHO_PIN}
};

const int sensorCount = sizeof(sensors) / sizeof(sensors[0]);

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
  }

  Serial.println("=== Test des capteurs ultrasoniques ===");
}

void loop() {
  for (int i = 0; i < sensorCount; i++) {
    float distance = readUltrasonic(sensors[i].trig, sensors[i].echo);
    Serial.print(sensors[i].name);
    Serial.print(" : ");
    if (distance == -1) {
      Serial.println("PAS DE SIGNAL");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }
  }

  Serial.println("----------------------");
  delay(1000);
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20 ms = max 3,4 m
  if (duration == 0) return -1; // Pas de signal reçu
  return duration * 0.0343 / 2;
}
