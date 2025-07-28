/**
 * @file sensor.h
 * @brief Módulo para inicializar y leer distancia del sensor VL53L0X (ej. TOF200C).
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

/**
 * @brief Inicializa el sensor VL53L0X.
 * 
 * @return true si fue exitoso, false si falló (no se detectó el sensor).
 */
bool iniciarSensor();

/**
 * @brief Lee la distancia medida por el sensor en centímetros.
 * 
 * @return float Distancia en centímetros. Si falla, devuelve -1.
 */
float leerDistanciaCM();

#endif
