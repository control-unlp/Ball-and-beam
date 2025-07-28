/*
----------------------------------------------------------
  Ball and Beam Control - Controlador Digital Discreto
                      UNLP          
----------------------------------------------------------
*/

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// ------------------------ SERVO ------------------------------
Servo myservo;      // create servo object to control a servo
int potpin = 3;     // analog pin used to connect the potentiometer
int val;            // variable to read the value from the analog pin

// ------------------------ SENSOR ------------------------------
VL53L0X sensor;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    myservo.attach(9);  // attaches the servo on pin 9 to the servo object
    myservo.write(112);   

    // ------------------------ SENSOR ------------------------------
    sensor.init();
    sensor.setTimeout(500);
    sensor.setMeasurementTimingBudget(20);

} 

void loop() {

    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo.write(val);                  // sets the servo position according to the scaled value
    
    // float DISTANCIA = getDISTANCIA(n_Samples);
    float DISTANCIA = sensor.readRangeSingleMillimeters()/10;

    if (sensor.timeoutOccurred()) {
        Serial.print("TIMEOUT");
    } else {
        Serial.print("DISTANCIA: ");
        Serial.print(DISTANCIA);
        Serial.println(" cm");
    }

    delay(10);                                 // waits for the servo to get there

}
