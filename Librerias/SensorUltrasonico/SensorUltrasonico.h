#ifndef SENSOR_ULTRASONICO_H
#define SENSOR_ULTRASONICO_H

// Incluye NewPing si lo vas a usar en este módulo
#include <NewPing.h>

// Prototipo de la función única
float medirUltrasonico(int trigPin, int echoPin, int mode);

#endif
