#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>  // Compatible con VL53L0X y algunos TOF200C

// ------------- CONFIGURACIÓN -----------------------
#define SENSOR_MODE 2  // 1 = Ultrasónico, 2 = VL53L0X o TOF200C

const int trigPin = 6;
const int echoPin = 5;
const int potPin = A3;
const int motorPin = 9;
const int maxDist = 50;

// Servo
Servo myServo;

// Ultrasónico
NewPing sonar(trigPin, echoPin, maxDist);

// VL53L0X / TOF200C
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ------------- SETUP ------------------------------
void setup() {
  Serial.begin(9600);
  myServo.attach(motorPin);

#if SENSOR_MODE == 1
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
#elif SENSOR_MODE == 2
  Wire.begin();
  if (!lox.begin()) {
    Serial.println("Fallo al iniciar VL53L0X/TOF200C. Verifique conexión.");
    while (1);
  }
#endif

  Serial.println("Ball and Beam PID+N Control");
}

// ------------- LOOP PRINCIPAL ---------------------
void loop() {
  int potValue = analogRead(potPin);
  int angle = map(potValue, 0, 1023, 50, 170);

  myServo.write(angle);

  float distance = readDistance();

  Serial.print(distance);
  Serial.print("\n");

  delay(50);
}

// ------------- LECTURA DISTANCIA ------------------

float readDistance() {
#if SENSOR_MODE == 1
  return readUltrasonicFiltered();
#elif SENSOR_MODE == 2
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus == 0) {
    return measure.RangeMilliMeter / 10.0; // mm → cm
  } else {
    return -1.0;  // Sin lectura válida
  }
#else
  return -1.0;
#endif
}

// --- Ultrasonido con filtrado básico
float readUltrasonicFiltered() {
  static float lastDistance = 0;
  float ms = sonar.ping_median(5, 500);
  float distance = sonar.convert_cm(ms);

  if (distance > 40 || distance == 0) return lastDistance;
  if (abs(distance - lastDistance) > 10) return lastDistance;

  lastDistance = distance;
  return distance;
}
