#include <Servo.h>
#include <NewPing.h>

// Pines
const int trigPin = 6;
const int echoPin = 5;
const int servoPin = 7;
const int maxDist = 50;

const int potPin = A5;
const int motorPin = 9;

// Sonar
NewPing sonar(trigPin, echoPin, maxDist);

// Servo
Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(motorPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Ball and Beam PID+N Control");
}

void loop() {

  int potValue = analogRead(potPin); 
  int angle = map(potValue, 0, 1023, 75, 170);

  // Mover el servo
  myServo.write(angle);

  float distance = readUltrasonic2();
  //Serial.print("Distancia: "); Serial.print(distance); Serial.print(" cm\n");
  Serial.print(distance);; Serial.print("\n");

}

float readUltrasonic() {
  static float lastDistance = 0;  // Guarda la última medición válida

  delay(40);
  float distance = sonar.ping_cm();

  // Límite superior
  if (distance > 40) {
    distance = 40;
  }

  // Si la lectura es 0 (sin respuesta del sensor), podés decidir ignorarla
  if (distance == 0) {
    return lastDistance;
  }

  // Filtrado: si el cambio es mayor a 10 cm, ignora y retorna la anterior
  if (abs(distance - lastDistance) > 10) {
    return lastDistance;
  }
  
  lastDistance = distance;
  return distance;
}

float readUltrasonic2() {

  float ms = sonar.ping_median(5 , 35);
  float distance = sonar.convert_cm(ms);
  return distance;
}