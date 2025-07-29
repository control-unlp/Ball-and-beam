#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <PID_v1.h>

// ------------------------ SERVO ------------------------------
Servo myservo;
int servoPos = 115;
int servoMin = 70;
int servoMax = 160;

// ------------------------ SENSOR ------------------------------
VL53L0X sensor;
const int rangoMin = 0;
const int rangoMax = 40;
const int distanciaError = -1;

// ------------------------ PID ------------------------------
double Setpoint, Input, Output;
double Kp = 0.4, Ki = 0, Kd = 0.07;
double angle = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

// ------------------------ FILTRO (media móvil) ------------------------------
#define N 3
float buffer[N];
int idx = 0;
bool bufferLleno = false;

float filtrar(float valor) {
  buffer[idx] = valor;
  idx = (idx + 1) % N;
  float suma = 0;
  int n = bufferLleno ? N : idx;
  for (int i = 0; i < n; i++) suma += buffer[i];
  return suma / n;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  myservo.attach(9);
  myservo.write(servoPos);

  sensor.init();
  sensor.setTimeout(100);
  sensor.setMeasurementTimingBudget(20000); // más lento pero más preciso

  Setpoint = 16;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(servoMin - servoPos, servoMax - servoPos);
  myPID.SetSampleTime(20);
  myPID.SetTunings(Kp, Ki, Kd);

  delay(3000); // espera de arranque
}

void loop() {
  int raw = sensor.readRangeSingleMillimeters();
  
  // 1. Detectar errores
  if (sensor.timeoutOccurred()) {
    Serial.println("ERROR_TIMEOUT");
    return;  // salta el ciclo sin mover el servo
  }

  float distancia = raw / 10.0; // en cm

  if (distancia < rangoMin || distancia > rangoMax || raw == 8190) {
    Serial.println("ERROR_OUT_OF_RANGE");
    return;
  }

  // 2. Filtro simple
  float distancia_filtrada = filtrar(distancia);

  // 3. PID
  Input = distancia_filtrada;
  float error = Input - Setpoint;

  myPID.Compute();
  angle = constrain(servoPos + Output, servoMin, servoMax);
  myservo.write(angle);

  // 4. Log
  Serial.print(distancia_filtrada);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.println(angle);

  delay(10);
}
