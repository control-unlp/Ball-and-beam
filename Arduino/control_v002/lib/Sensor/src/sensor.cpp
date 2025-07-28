#include "sensor.h"
#include <Wire.h>
#include <VL53L0X.h>

static VL53L0X sensor;

bool iniciarSensor() {
    Wire.begin();

    sensor.setTimeout(500);

    if (!sensor.init()) {
        // Sensor no encontrado
        return false;
    }

    sensor.setMeasurementTimingBudget(20000); // 20 ms -> buena precisión
    sensor.startContinuous(); // modo continuo
    return true;
}

float leerDistanciaCM() {
    uint16_t lectura = sensor.readRangeContinuousMillimeters();

    if (sensor.timeoutOccurred()) {
        return -1.0f;
    }

    return lectura / 10.0f; // mm a cm
}
