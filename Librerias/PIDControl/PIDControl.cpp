#include "PIDControl.h"
#include <Arduino.h>

// Variables estáticas para que el PID de la librería recuerde estado
static double outputLib = 0;
static double inputLib = 0;
static double setpointLib = 0;
static PID pidLib(&inputLib, &outputLib, &setpointLib, 1, 0, 0, DIRECT);
static bool pidInitialized = false;

// Variables para PID propio (acumulador de error)
static float integralError = 0;
static float lastError = 0;

float calcularPID(
    float Kp,
    float Ki,
    float Kd,
    float input,
    float referencia,
    int mode
) {
    float output = 0;

    switch (mode) {
        case 0:
            // PID propio
            {
                float error = referencia - input;
                integralError += error;
                float derivative = error - lastError;
                output = Kp * error + Ki * integralError + Kd * derivative;
                lastError = error;
            }
            break;

        case 1:
            // PID librería
            {
                if (!pidInitialized) {
                    pidLib.SetMode(AUTOMATIC);
                    pidInitialized = true;
                }
                // Update tunings each call in case they change
                pidLib.SetTunings(Kp, Ki, Kd);

                inputLib = input;
                setpointLib = referencia;

                pidLib.Compute();

                output = outputLib;
            }
            break;

        default:
            output = 0;
            break;
    }

    return output;
}
