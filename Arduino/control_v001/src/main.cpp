/*
----------------------------------------------------------
  Ball and Beam Control - Controlador Digital Discreto
                      UNLP          
----------------------------------------------------------
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <PID_v1.h>

// ------------------ Servo ------------------
Servo servoMotor;
const int pinServo = 9;
const float offset = 120.0;
const float servoMin = 75.0;
const float servoMax = 170.0;

// ------------------ Sensor ------------------
VL53L0X sensor;

// ------------------ PID ------------------
float Kp = 1.1;                                                   //Initial Proportional Gain 2.05
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 0.4;                                                    //Intitial Derivative Gain 0.85
double Setpoint, Input, Output, ServoOutput;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.                                                                   
Servo myServo;                                                       //Initialize Servo.

// ------------------ Setup ------------------
void setup() {
    Serial.begin(9600);
    Wire.begin();
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Sensor no detectado!");
        while (1);
    }

    sensor.setMeasurementTimingBudget(20000);
    sensor.startContinuous();

    Input = sensor.readRangeContinuousMillimeters() / 10.0;

    servoMotor.attach(pinServo);

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-50, 50);  // grados desde el offset
}

// ------------------ Loop ------------------
void loop() {

    delay(100);
    Setpoint = 16.5;
    Input = sensor.readRangeContinuousMillimeters() / 10.0;

    myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  
    ServoOutput=120-Output;                                            // 102 degrees is my horizontal 

    if (ServoOutput > servoMax) ServoOutput = servoMax;
    if (ServoOutput < servoMin) ServoOutput = servoMin;
    servoMotor.write(ServoOutput);                                        //Writes value of Output to servo
    Serial.print("angle: ");
    Serial.println(ServoOutput);

}
