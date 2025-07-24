/**
 * @file control.h
 * @brief Control discreto para Ball and Beam basado en diferencias de estados.
 */

#ifndef CONTROL_H
#define CONTROL_H

/**
 * @brief Estructura de estado para el controlador discreto.
 */
struct ControlState {
    float e_hist;  ///< Error anterior
    float u_hist;  ///< Salida anterior
};

/**
 * @brief Inicializa la estructura de estado.
 * @param state Referencia a la estructura a inicializar.
 */
void initControlState(ControlState &state);

/**
 * @brief Calcula la salida del controlador discreto.
 * 
 * @param ref Referencia deseada (cm).
 * @param distancia Medición actual (cm).
 * @param a Coeficientes del denominador [a0=1, a1].
 * @param b Coeficientes del numerador [b0, b1].
 * @param state Estado del controlador (errores y salidas pasadas).
 * @return float Salida de control `u`.
 */
float calcularControl(float ref, float distancia, const float a[2], const float b[2], ControlState &state);

// ------ Control PIDN (modificable por potenciómetros) ------

struct PIDN_Params {
    float Kp;
    float Ki;
    float Kd;
    float N;    // Filtro derivativo
};

struct PIDN_State {
    float error_prev;
    float integral;
    float derivada_filtrada;
    unsigned long last_time;
};

void initPIDNState(PIDN_State &state);
PIDN_Params leerPIDNDesdePotenciometros(int pinKp, int pinKi, int pinKd, int pinN, float maxKp, float maxKi, float maxKd, float maxN);
float calcularPIDN(float ref, float medida, const PIDN_Params& params, PIDN_State &state);

#endif
