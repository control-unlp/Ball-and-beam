#include <Servo.h>

Servo motor;
const int potPin = A5;
const int motorPin = 7;

void setup() {
  Serial.begin(9600);
  motor.attach(motorPin);
  Serial.println("Test de potenciómetro y motor");
}

void loop() {
  // Leer el potenciómetro
  int potValue = analogRead(potPin); // 0 - 1023

  // Mapear a ángulo de servo
  int angle = map(potValue, 0, 1023, 75, 170);

  // Mover el servo
  motor.write(angle);

  // Mostrar los valores
  Serial.print("PotValue: ");
  Serial.print(potValue);
  Serial.print("\tÁngulo: ");
  Serial.println(angle);

  delay(50); // Para que sea legible en el monitor serial
}
