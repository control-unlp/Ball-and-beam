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
#include <PID_v1.h>

// ------------------------ SERVO ------------------------------
Servo myservo;      // create servo object to control a servo
int servoPos = 115; 
int servoMin = 70; 
int servoMax = 160; 

// ------------------------ SENSOR ------------------------------
VL53L0X sensor;

// ------------------------ PID  ------------------------------
double Setpoint, Input, Output;
double Kp=0.4, Ki=0, Kd=0.07; 
double angle = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);  

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // ------------------------ MOTOR ------------------------------
    myservo.attach(9);  // attaches the servo on pin 9 to the servo object
    myservo.write(servoPos);   

    // ------------------------ SENSOR ------------------------------
    sensor.init();
    sensor.setTimeout(500);

    // ------------------------ PID  ------------------------------
    Setpoint = 16; 
    myPID.SetMode(AUTOMATIC);
    
    myPID.SetOutputLimits(servoMin - servoPos, servoMax - servoPos); // limits the output to the servo
    //myPID.SetOutputLimits(servoMax - servoPos, servoMin - servoPos); // limits the output to the servo
    myPID.SetSampleTime(20); // sets the sample time to 10 ms
    myPID.SetTunings(Kp, Ki, Kd); // sets the PID tunings

    delay(6000); // wait for the sensor to initialize
} 

void loop() {

               
    float DISTANCIA = sensor.readRangeSingleMillimeters()/10;

    Input = DISTANCIA; // read the distance from the sensor
    float error = Input - Setpoint; // calculate the error

    myPID.Compute(); // compute the PID output
    angle = servoPos + Output; // calculate the angle for the servo
    myservo.write(angle); // write the output to the servo

    Serial.print("Input: ");
    Serial.print(Input);    
    Serial.print(" cm, Error: ");    
    Serial.print(error); // print the error to the serial monitor
    Serial.print(" cm, Output: ");
    Serial.println(angle); // print the output to the serial monitor   

    delay(10);                                 // waits for the servo to get there

}
