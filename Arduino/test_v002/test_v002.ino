#include <Servo.h>

// ---------------------------
// Pines del sensor ultrasónico
// ---------------------------
const int trig = 6;
const int echo = 5;

// ---------------------------
// Servo motor
// ---------------------------
Servo servoMotor;
const int servoPin = 7;

// ---------------------------
// Potenciómetros para Kp, Ki, Kd
// ---------------------------
const int KpPin = A5;
const int KiPin = A4;
const int KdPin = A3;

// ---------------------------
// Parámetros físicos
// ---------------------------
const float ref = 6;            // Referencia deseada en cm
const float vel_sonido = 0.034;   // cm/us

// ---------------------------
// Límites de la salida al servo
// ---------------------------
const int angulo_min = 50;
const int angulo_max = 170;

// ---------------------------
// Máximos de las ganancias
// ---------------------------
const float Kp_max = 5.0;
const float Ki_max = 1.0;
const float Kd_max = 2.0;

// ---------------------------
// Variables de control PID
// ---------------------------
float Kp, Ki, Kd;
float error = 0;
float error_previo = 0;
float integral = 0;
float derivada = 0;
float control = 0;

// ---------------------------
// Tiempos
// ---------------------------
unsigned long t_muestreo_anterior = 0;
float dt = 0;

// ---------------------------
// Tasa de muestreo
// ---------------------------
const unsigned long periodo_muestreo_ms = 100; // 50 ms = 20 Hz

void setup() {
  Serial.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);

  servoMotor.attach(servoPin);
  servoMotor.write(170); // Posición neutra

  delay(3000); // Pequeña estabilización
}

void loop() {
  unsigned long t_actual = millis();

  // Verificar si es momento de muestrear
  if (t_actual - t_muestreo_anterior >= periodo_muestreo_ms) {
    // Calcular delta t en segundos
    dt = (t_actual - t_muestreo_anterior) / 1000.0;
    t_muestreo_anterior = t_actual;

    // Leer distancia
    float posicion = medirDistancia(trig, echo);

    // Leer ganancias de potenciómetros
    Kp = analogRead(KpPin) / 1023.0 * Kp_max;
    Ki = analogRead(KiPin) / 1023.0 * Ki_max;
    Kd = analogRead(KdPin) / 1023.0 * Kd_max;

    // Calcular error
    error = ref - posicion;

    // Calcular PID
    integral += error * dt;
    derivada = (error - error_previo) / dt;
    control = Kp * error + Ki * integral + Kd * derivada;

    // Invertir sentido si necesario
    control = -control;

    // Guardar error previo
    error_previo = error;

    // Calcular ángulo de salida
    int angulo = 90 + control;
    angulo = constrain(angulo, angulo_min, angulo_max);

    // Mover servo
    servoMotor.write(angulo);

    // Enviar datos al Serial Plotter
    // Formato: Tiempo,Referencia,Posicion,Angulo
    Serial.print(t_actual / 1000.0, 3); // Tiempo en segundos con 3 decimales
    Serial.print(",");
    Serial.print(ref);
    Serial.print(",");
    Serial.print(posicion);
    Serial.print(",");
    Serial.println(angulo);
  }
}

// ---------------------------
// Medir distancia con ultrasonido
// ---------------------------
float medirDistancia(int trigPin, int echoPin) {
  // Enviar pulso
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Medir duración del eco
  long duracion = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout

  // Si no hubo rebote, devolver última distancia válida o un valor grande
  if (duracion == 0) {
    return 999.0;
  }

  // Convertir a cm
  float distancia = (duracion * vel_sonido) / 2.0;
  return distancia;
}
