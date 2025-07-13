#include <Servo.h>

// Pines
const int trigPin = 6;
const int echoPin = 5;
const int servoPin = 7;

const int potP = A5;
const int potI = A4;
const int potD = A3;
const int potN = A2;

// Servo
Servo myServo;

// Setpoint
const float y_ref = 10; // cm

// PID variables
float error = 0;
float lastError = 0;
float lastDeriv = 0;
float integral = 0;

// Parámetros
float Kp = 0;
float Ki = 0;
float Kd = 0;
float Kn = 0;

// Tiempo
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Ball and Beam PID+N Control");
}

void loop() {
  // Leer tiempo
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // segundos
  if (dt <= 0) dt = 0.01; // evitar división por cero
  lastTime = currentTime;

  // Leer potenciómetros y mapear
  Kp = analogRead(potP) / 1023.0 * 10.0; // 0 - 10
  Ki = analogRead(potI) / 1023.0 * 5.0;  // 0 - 5
  Kd = analogRead(potD) / 1023.0 * 5.0;  // 0 - 5
  Kn = analogRead(potN) / 1023.0 * 2.0;  // 0 - 2

  // Leer distancia
  float distance = readUltrasonic();

  // Calcular error
  error = y_ref - distance;

  // Integral
  integral += error * dt;

  // Derivada
  float deriv = (error - lastError) / dt;

  // Segunda derivada (aceleración del error)
  float deriv2 = (deriv - lastDeriv) / dt;

  // PID+N
  float control = Kp * error + Ki * integral + Kd * deriv + Kn * deriv2;

  // Mapear control a ángulo de servo
  // Suponemos que el ángulo debe estar entre 60 y 120 grados
  int servoAngle = constrain(mapFloat(control, -20, 20, 120, 60), 60, 120);
  myServo.write(servoAngle);

  // Guardar errores previos
  lastError = error;
  lastDeriv = deriv;

  // Mostrar datos
  Serial.print("Distancia: "); Serial.print(distance); Serial.print(" cm");
  Serial.print(" | P: "); Serial.print(Kp, 2);
  Serial.print(" I: "); Serial.print(Ki, 2);
  Serial.print(" D: "); Serial.print(Kd, 2);
  Serial.print(" N: "); Serial.print(Kn, 2);
  Serial.print(" | Control: "); Serial.print(control, 2);
  Serial.print(" | Servo: "); Serial.println(servoAngle);

  delay(50);
}

// Función para medir distancia con ultrasónico
float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  float distanceCm = duration * 0.0343 / 2;
  return distanceCm;
}

// Mapear flotante
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
