#include "control.h"
#include <Arduino.h>

void initControlState(ControlState &state) {
    state.e_hist = 0.0f;
    state.u_hist = 0.0f;
}

float calcularControl(float ref, float distancia, const float a[2], const float b[2], ControlState &state) {
    float e = ref - distancia;
    float u = b[0]*e + b[1]*state.e_hist - a[1]*state.u_hist;

    // Actualizar historial
    state.e_hist = e;
    state.u_hist = u;

    return u;
}

// ----------- CONTROL PIDN -----------

void initPIDNState(PIDN_State &state) {
    state.error_prev = 0.0f;
    state.integral = 0.0f;
    state.derivada_filtrada = 0.0f;
    state.last_time = millis();
}

PIDN_Params leerPIDNDesdePotenciometros(int pinKp, int pinKi, int pinKd, int pinN, float maxKp, float maxKi, float maxKd, float maxN) {
    PIDN_Params params;
    params.Kp = analogRead(pinKp) * maxKp / 1023.0;
    params.Ki = analogRead(pinKi) * maxKi / 1023.0;
    params.Kd = analogRead(pinKd) * maxKd / 1023.0;
    params.N  = analogRead(pinN)  * maxN  / 1023.0;
    return params;
}

float calcularPIDN(float ref, float medida, const PIDN_Params& params, PIDN_State &state) {
    float error = ref - medida;
    unsigned long t = millis();
    float dt = (t - state.last_time) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;

    state.integral += error * dt;

    float derivada = (error - state.error_prev) / dt;
    state.derivada_filtrada += dt * params.N * (derivada - state.derivada_filtrada);

    float salida = params.Kp * error + params.Ki * state.integral + params.Kd * state.derivada_filtrada;

    state.error_prev = error;
    state.last_time = t;

    return salida;
}

