#include "SensorUltrasonico.h"
#include <Arduino.h>

// Opcional: definir parámetros de NewPing
#define MAX_DISTANCE_CM 400

float medirUltrasonico(int trigPin, int echoPin, int mode) {
  float distancia = 0;

  switch (mode) {
    case 0:
      // Modo 0: medición básica con pulseIn
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      long duracion = pulseIn(echoPin, HIGH, MAX_DISTANCE_CM * 58);
      distancia = duracion * 0.034 / 2;
      break;

    case 1:
      // Modo 1: medición con NewPing
      {
        NewPing sonar(trigPin, echoPin, MAX_DISTANCE_CM);
        distancia = sonar.ping_cm();
      }
      break;

    case 2:
      // Modo 2: medición filtrada (ejemplo: promedio de 5)
      {
        float suma = 0;
        int repeticiones = 5;
        for (int i = 0; i < repeticiones; i++) {
          digitalWrite(trigPin, LOW);
          delayMicroseconds(2);
          digitalWrite(trigPin, HIGH);
          delayMicroseconds(10);
          digitalWrite(trigPin, LOW);
          long d = pulseIn(echoPin, HIGH, MAX_DISTANCE_CM * 58);
          suma += d * 0.034 / 2;
          delay(20);
        }
        distancia = suma / repeticiones;
      }
      break;

    default:
      distancia = -1; // error
      break;
  }

  return distancia;
}
