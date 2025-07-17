/*
----------------------------------------------------------
  Ball and Beam Control - Controlador Digital Discreto
                      UNLP          
----------------------------------------------------------
*/

#include <NewPing.h>
#include <Servo.h>

// ------------- CONFIGURACIÓN ---------------------------

#define TRIGGER_PIN  6          // Pin trigger ultrasonido
#define ECHO_PIN     5          // Pin echo ultrasonido
#define MAX_DISTANCE 200        // Distancia máxima (cm)

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

Servo servoMotor;

// ----------------- CONTROL ----------------------------

const float Ts = 0.4;           // Tiempo de muestreo 

float a[2] = {2.0215, -1.8913}; 

float b[2] = {1.0, -0.6509};       

unsigned long lastTime = 0;     // Tiempo 
float referencia_cm = 16.0;     // Referencia 

float e_hist[1] = {0};          // e[k-i]
float u_hist[1] = {0};          // u[k-i]

// ----------------- MOTOR -----------------------------

const float Offset = 120.0;     // Punto medio

const float servoMin = 75.0;    // Mínimo
const float servoMax = 170.0;   // Máximo

void setup() {
  Serial.begin(9600);
  servoMotor.attach(9);
  servoMotor.write(Offset);
  Serial.println("Control discreto inicializado.");
}

void loop() {
  unsigned long now = millis();
  if ((now - lastTime) >= Ts * 1000.0) {
    lastTime = now;

    // Medir distancia
    float medida_cm = sonar.ping_cm();
    if (medida_cm == 0) medida_cm = referencia_cm; // Si falla medición

    // Error actual
    float e0 = referencia_cm - medida_cm;

    // --------------------------
    // ECUACIÓN EN DIFERENCIAS
    // --------------------------

    // Calcular numerador
    float num = a[0]*e0 + a[1]*e_hist[0];

    // Calcular denominador
    float den = b[1]*u_hist[0];

    // Salida actual
    float u0 = (num - den) / b[0];

    // Actualizar historiales
    e_hist[0] = e0;
    u_hist[0] = u0;

    // Convertir a grados + offset
    float theta_deg = u0 + Offset;

    // Saturar ángulo y mandar a motor
    if (theta_deg > servoMax) theta_deg = servoMax;
    if (theta_deg < servoMin) theta_deg = servoMin;
    servoMotor.write(theta_deg);

    // Debug
    Serial.print("Ref: ");
    Serial.print(referencia_cm);
    Serial.print(" cm | Medida: ");
    Serial.print(medida_cm);
    Serial.print(" cm | Error: ");
    Serial.print(e0);
    Serial.print(" | Theta: ");
    Serial.println(theta_deg);
  }
}


