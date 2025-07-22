#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>  // TOF sensor (si el TOF200C es compatible)

#define SERVO_PIN 9
#define POT_PIN A3

const int anguloMin = 50;
const int anguloMax = 170;

Servo servoMotor;
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  servoMotor.attach(SERVO_PIN);

  // Iniciar sensor TOF
  if (!tof.begin()) {
    Serial.println("Fallo al iniciar el sensor TOF. Chequear conexiones.");
    //while (1);
  }

  delay(500);
  Serial.println("Sensor TOF y servo listos.");
}

void loop() {
  // Leer valor del potenciómetro (0-1023)
  int valorPot = analogRead(POT_PIN);

  // Mapear a rango del servo (50° a 170°)
  int anguloServo = map(valorPot, 0, 1023, anguloMin, anguloMax);

  // Mover servo
  servoMotor.write(anguloServo);

  // Leer distancia del TOF
  VL53L0X_RangingMeasurementData_t medida;
  tof.rangingTest(&medida, false);

  if (medida.RangeStatus == 0) {
    Serial.print("Distancia [mm]: ");
    Serial.print(medida.RangeMilliMeter);
    Serial.print("\tÁngulo servo: ");
    Serial.println(anguloServo);
  } else {
    Serial.println("Error en la medición TOF.");
  }

  delay(200);
}
