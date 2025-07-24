/*
----------------------------------------------------------
  Ball and Beam Control - Controlador Digital Discreto
                      UNLP          
----------------------------------------------------------
*/

#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include "sensor.h"  // Tu librería ya funcional

// ------------------- Servo -------------------
Servo servoMotor;
const int pinServo = 9;
const float Offset = 120.0;   // Punto medio
const float servoMin = 75.0;
const float servoMax = 170.0;

// ------------------- Control -------------------
double ref = 16.0;     // [cm]
double input = 0.0;
double output = 0.0;

double Kp = 2.0, Ki = 0.0, Kd = 1.0;
PID pid(&input, &output, &ref, Kp, Ki, Kd, DIRECT);

// ------------------- Potenciómetros -------------------
const int pinKp = A3;
const int pinKi = A2;
const int pinKd = A1;

const float maxKp = 5.0;
const float maxKi = 1.0;
const float maxKd = 2.0;

// ------------------- Muestreo -------------------
const float Ts = 0.4;
unsigned long t_prev = 0;

void setup() {
    Serial.begin(9600);
    servoMotor.attach(pinServo);

    if (!iniciarSensor()) {
        Serial.println("Sensor no detectado.");
        while (1);
    }

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-45, 45);  // Salida acotada en grados respecto al offset
    t_prev = millis();
}

void loop() {
    unsigned long t_now = millis();
    if ((t_now - t_prev) < Ts * 1000) return;
    t_prev = t_now;

    // Leer distancia
    float distancia = leerDistanciaCM();
    if (distancia < 0) {
        Serial.println("Error en sensor");
        return;
    }

    // Leer constantes PID
    Kp = analogRead(pinKp) * maxKp / 1023.0;
    Ki = analogRead(pinKi) * maxKi / 1023.0;
    Kd = analogRead(pinKd) * maxKd / 1023.0;
    pid.SetTunings(Kp, Ki, Kd);

    // Actualizar entrada y calcular
    input = distancia;
    pid.Compute();

    // Mapear salida a ángulo
    float theta = output + Offset;
    if (theta > servoMax) theta = servoMax;
    if (theta < servoMin) theta = servoMin;

    servoMotor.write(theta);

    // Debug
    Serial.print("Error: ");
    Serial.print(ref - input, 2);
    Serial.print(" | u: ");
    Serial.print(theta, 2);
    Serial.print(" | Distancia: ");
    Serial.print(input, 2);
    Serial.println(" cm");
}
