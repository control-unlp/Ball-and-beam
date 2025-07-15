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
const float setpoint = 15;

Servo servoMotor;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servoMotor.attach(9);
}

void loop() {

  float suma = 0;
  int repeticiones = 5;
  for (int i = 0; i < repeticiones; i++) {
    float d = medirUltrasonico(trigPin, echoPin, 0);
    suma += d * 0.034 / 2;
    delay(20);
  }

  float medida = suma *100 / repeticiones;

  float Kp = analogRead(kpPin) / 1023.0 * 10.0;   // Escalar 0–10
  float Ki = analogRead(kiPin) / 1023.0 * 3.0;    // Escalar 0–1
  float Kd = analogRead(kdPin) / 1023.0 * 3.0;    // Escalar 0–1

  // Calcular PID (modo 0 = PID propio)
  float output = calcularPID(Kp, Ki, Kd, medida, setpoint, 0);

  int anguloMotor = mapPIDtoAngle(output);
  servoMotor.write(anguloMotor);

  // Mostrar resultados
  Serial.print("Distancia: ");
  Serial.print(medida);
  Serial.print(" cm  |  PID Output: ");
  Serial.print(output);
  Serial.print("  |  Kp=");
  Serial.print(Kp, 2);
  Serial.print(" Ki=");
  Serial.print(Ki, 2);
  Serial.print(" Kd=");
  Serial.println(Kd, 2);

  delay(200);
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

