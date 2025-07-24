/*
----------------------------------------------------------
  Ball and Beam Control - Controlador Digital Discreto
                      UNLP          
----------------------------------------------------------
*/

#include <Arduino.h>
#include <Servo.h>
#include "control.h"
#include "sensor.h"

// ------------- CONFIGURACIÓN ---------------------------
#define CONTROL_MODE CONTROL_PIDN   // o CONTROL_DISCRETO

// Servo
Servo servoMotor;
const int pinServo = 9;

// Referencia
const float referencia_cm = 16.0;   // cm

// Muestreo
const float Ts = 0.4;               // [s]
unsigned long t_anterior = 0;

// Offset y saturación
const float Offset = 120.0;         // Punto medio (grados)
const float servoMin = 75.0;
const float servoMax = 170.0;

#if CONTROL_MODE == CONTROL_DISCRETO
    const float a[2] = {1.0, -1.8913};
    const float b[2] = {1.0, -0.6509};
    ControlState controlState;

#elif CONTROL_MODE == CONTROL_PIDN
    const int pinKp = A3;
    const int pinKi = A2;
    const int pinKd = A1;
    const int pinN  = A0;

    const float maxKp = 5.0;
    const float maxKi = 1.0;
    const float maxKd = 2.0;
    const float maxN  = 20.0;

    PIDN_Params pidnParams;
    PIDN_State pidnState;
#endif


void setup() {
    Serial.begin(9600);

    servoMotor.attach(pinServo);

    if (!iniciarSensor()) {
        Serial.println("Error: Sensor VL53L0X no detectado.");
        while (1);
    }

    #if CONTROL_MODE == CONTROL_DISCRETO
        initControlState(controlState);
    #elif CONTROL_MODE == CONTROL_PIDN
        initPIDNState(pidnState);
    #endif

    t_anterior = millis();
}

void loop() {
    unsigned long t_actual = millis();
    float dt = (t_actual - t_anterior) / 1000.0;

    if (dt >= Ts) {
        t_anterior = t_actual;

        // Leer sensor
        float distancia = leerDistanciaCM();

        //if (distancia < 0) {
        //    Serial.println("Lectura inválida del sensor");
        //    return;
        //}

        // Calcular control
        float u0 = 0.0;
        float error = referencia_cm - distancia;

        #if CONTROL_MODE == CONTROL_DISCRETO
            u0 = calcularControl(referencia_cm, distancia, a, b, controlState);

        #elif CONTROL_MODE == CONTROL_PIDN
            pidnParams = leerPIDNDesdePotenciometros(pinKp, pinKi, pinKd, pinN, maxKp, maxKi, maxKd, maxN);
            u0 = calcularPIDN(referencia_cm, distancia, pidnParams, pidnState);
        #endif

        // Mapear a ángulo de servo
        float theta_deg = -u0 + Offset;

        // Saturar
        if (theta_deg > servoMax) theta_deg = servoMax;
        if (theta_deg < servoMin) theta_deg = servoMin;

        // Mover servo
        servoMotor.write(theta_deg);

        // Imprimir datos
        Serial.print("Error: ");
        Serial.print(error, 2);
        Serial.print(" | u: ");
        Serial.print(theta_deg, 2);
        Serial.print(" | Distancia: ");
        Serial.print(distancia, 2);
        Serial.println(" cm");
    }
}
