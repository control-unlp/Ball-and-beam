#include <Servo.h>
#include <NewPing.h>

// Pines
const int trigPin = 6;
const int echoPin = 5;
const int servoPin = 7;
const int maxDist = 50;

const int potPin = A5;
const int motorPin = 7;

// Sonar
NewPing sonar(trigPin, echoPin, maxDist);

// Servo
Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(motorPin);
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);

  Serial.println("Ball and Beam PID+N Control");
}

void loop() {

  int potValue = analogRead(potPin); 
  int angle = map(potValue, 0, 1023, 75, 170);

  // Mover el servo
  myServo.write(angle);

  float distance = readUltrasonic();
  Serial.print("Distancia: "); Serial.print(distance); Serial.print(" cm\n");

}

float readUltrasonic() {
  delay(40);                                                            
  long distance;
  distance = sonar.convert_cm(sonar.ping_median(5));
  
  if(distance > 40){     // 40 cm is the maximum position for the ball
    distance = 40;
  }

  return distance;    
}