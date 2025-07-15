#include <PID_v1.h>
#include <Servo.h>
#include <NewPing.h>

#include "SensorUltrasonico.h"
#include "PIDControl.h"

// Pines de ultrasonido
const int trigPin = 6;
const int echoPin = 5;

// Pines de potenciómetros
const int kpPin = A5;
const int kiPin = A4;
const int kdPin = A3;

// Setpoint deseado (en cm)
const float setpoint = 16;

// Global
static float medidaFiltrada = 0;
static float outputFiltrado = 0;
static float ultimoAngulo = 90;

Servo servoMotor;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servoMotor.attach(9);
}

void loop() {
 static float lastDistance = 0;

  const int repeticiones = 5;
  float suma = 0;

  for (int i = 0; i < repeticiones; i++) {
    float distancia = medirUltrasonico(trigPin, echoPin, 0); // ya en cm
    //if (abs(distancia - lastDistance) > 15) {
    //  distancia = lastDistance;
    //}
    suma += distancia;
    delay(20);
  }

  float medida = suma / repeticiones;
  medidaFiltrada = 0.9 * medidaFiltrada + 0.1 * medida;

  float Kp = analogRead(kpPin) / 1023.0 * 6.0;   // Escalar 0–10
  float Ki = analogRead(kiPin) / 1023.0 * 4.0;    // Escalar 0–1
  float Kd = analogRead(kdPin) / 1023.0 * 4.0;    // Escalar 0–1

  int anguloMotor = mapPIDtoAngle(outputFiltrado);

  // Limitar la rampa
  float maxDelta = 2.0;
  if (anguloMotor > ultimoAngulo + maxDelta) {
    anguloMotor = ultimoAngulo + maxDelta;
  } else if (anguloMotor < ultimoAngulo - maxDelta) {
    anguloMotor = ultimoAngulo - maxDelta;
  }

  ultimoAngulo = anguloMotor;

  servoMotor.write(anguloMotor);
  // Mostrar resultados
  Serial.print("Distancia: ");
  Serial.print(medida);
  Serial.print(" cm  |  PID Output: ");
  Serial.print(outputFiltrado);
  Serial.print("  |  Kp=");
  Serial.print(Kp, 2);
  Serial.print(" Ki=");
  Serial.print(Ki, 2);
  Serial.print(" Kd=");
  Serial.println(Kd, 2);

  delay(500);
}

int mapPIDtoAngle(float pidOutput) {
  // Rango esperado de salida PID
  const float minPID = -100;
  const float maxPID = +100;

  // Limitar al rango
  pidOutput = constrain(pidOutput, minPID, maxPID);

  // Mapear inverso: +100→50°, -100→170°
  float angulo = map(pidOutput, minPID, maxPID, 170, 50);

  // Limitar por seguridad
  angulo = constrain(angulo, 50, 170);

  return (int)angulo;
}

