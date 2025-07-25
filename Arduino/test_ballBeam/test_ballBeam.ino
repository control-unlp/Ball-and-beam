#include <Servo.h>
#include <NewPing.h>

// Pines del sensor ultrasónico
const int trig = 6;
const int echo = 5;

// Servo
Servo servoMotor;
const int servoPin = 7;

// Potenciometros
const int KpPin = A5;
const int KiPin = A4;
const int KdPin = A3;

// Parámetros físicos
const float largo_riel = 35.2; // cm
const float ref = 6;

// Velocidad del sonido
const float vel_sonido = 0.034; // cm/us

// Parámetros del control
float Kp, Ki, Kd;
float error, error_previo = 0;
float integral = 0;
float derivada, control;
const int Kp_max = 5.0;
const int Ki_max = 1.0;
const int Kd_max = 2.0;

//Parámetros tiempo
unsigned long t_previo = 0;
float dt ;

//Angulo servo
const int angulo_min = 70;
const int angulo_max = 150;

NewPing sonar(trig, echo, 50);

void setup() {
  Serial.begin(9600);
  
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);

  servoMotor.attach(servoPin);
  servoMotor.write(90);

  delay(100); // tiempo para estabilizar
}

void loop() {
  // Tiempo
  unsigned long t = millis(); // tiempo actual
  dt = (t - t_previo)/ 1000.0; // segs
  t_previo = t;

  // Leer distancia
  float posicion = readUltrasonic();

  // Leer potenciometro
  Kp = analogRead(KpPin) / 1023.0 * Kp_max; 
  Ki = analogRead(KiPin) / 1023.0 * Ki_max; 
  Kd = analogRead(KdPin) / 1023.0 * Kd_max; 

  // Error
  error = ref - posicion;

  //PID
  integral += error * dt;
  derivada = (error - error_previo)/dt;
  control = Kp*error + Ki*integral + Kd*derivada;
  control = -control;

  error_previo = error;
  
  //Mapeo control a angulo del servo
  int angulo = 90 + control;
  angulo = constrain(angulo, angulo_min, angulo_max);
  servoMotor.write(angulo);

  // Mostrar posición en el Serial Plotter
  Serial.print("Posicion: "); Serial.print(posicion);
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" Kp: "); Serial.print(Kp);
  Serial.print(" Ki: "); Serial.print(Ki);
  Serial.print(" Kd: "); Serial.print(Kd);
  Serial.print(" Servo: "); Serial.println(angulo);
  delay(300); // muestreo a ~20 Hz
}

// Función para medir la distancia con un sensor ultrasónico
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
