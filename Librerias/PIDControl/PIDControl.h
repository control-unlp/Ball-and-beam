#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <PID_v1.h> // Solo si quieres usar el PID oficial

float calcularPID(
    float Kp,
    float Ki,
    float Kd,
    float input,
    float referencia,
    int mode
);

#endif