#include <NewPing.h>
#include <Servo.h>
#include <PID_v1.h>

// Pines del sensor ultrasónico
const int trigPin = 6;
const int echoPin = 5;
const int maxDistance = 50;  // Máxima distancia de medición en cm

// Objeto del sensor ultrasónico
NewPing sonar(trigPin, echoPin, maxDistance);

// Servo motor
Servo servoMotor;
const int servoPin = 7;

// Potenciómetros (para Kp, Ki, Kd)
const int KpPin = A5;
const int KiPin = A4;
const int KdPin = A3;

// Variables PID
double input;        // Lectura del sensor
double output;       // Salida del PID (ángulo del servo)
double setpoint = 16.0;  // Posición deseada en cm

// Constantes PID (se actualizarán dinámicamente)
double Kp = 2.0, Ki = 0.5, Kd = 1.0;

// Rango del servo
const int servoMin = 80;
const int servoMax = 110;

// Crear el PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // Iniciar servo
  servoMotor.attach(servoPin);

  // Configurar PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(servoMin, servoMax);

  Serial.println("Sistema iniciado.");
}

void loop() {
  // Leer distancia del sensor
  delay(30); // Pequeña pausa entre mediciones
  input = sonar.ping_cm();

  if (input == 0) {
    // Si no detecta, mantener último ángulo
    input = setpoint;
  }

  // Leer potenciómetros y escalar a rangos
  Kp = map(analogRead(KpPin), 0, 1023, 0, 500) / 100.0;  // 0 - 5.0
  Ki = map(analogRead(KiPin), 0, 1023, 0, 100) / 100.0;  // 0 - 1.0
  Kd = map(analogRead(KdPin), 0, 1023, 0, 200) / 100.0;  // 0 - 2.0

  // Actualizar constantes del PID
  myPID.SetTunings(Kp, Ki, Kd);

  // Calcular PID
  myPID.Compute();

  // Mover servo
  servoMotor.write(output);

  // Mostrar valores por monitor serial
  Serial.print("Distancia: ");
  Serial.print(input);
  Serial.print(" cm | Kp: ");
  Serial.print(Kp);
  Serial.print(" Ki: ");
  Serial.print(Ki);
  Serial.print(" Kd: ");
  Serial.print(Kd);
  Serial.print(" | Servo: ");
  Serial.println(output);

  delay(50); // Ajusta si necesitas más rapidez
}

