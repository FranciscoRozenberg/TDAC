#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// PINES
#define PWM_PIN 5
#define PI 3.14159265359

// CREACION DEL SERVO
Servo servo;
// CREACION DE LA IMU
Adafruit_MPU6050 mpu;

// VARIABLES
const int posicion_inicial = 1500;       // us
const unsigned long T_MUESTREO = 20000;  // 20ms = 50 Hz
float sesgo_x = 0;

// MATRICES DE ESTADOS
// Vector de estado estimado X_hat (2 filas x 1 columna)
Matrix<2, 1> X_k = { 0.0,0.0 };
Matrix<1, 2> mat_K = {-3.9162  , -0.2581};

// SETUP
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // SERVO 
  servo.attach(PWM_PIN);
  servo.writeMicroseconds(posicion_inicial); // Posición inicial

  // IMU
  if (!mpu.begin()) {
    Serial.println("NO SE ENCONTRO EL MPU6050");
    while (1) {
      delay(10);
    }
  }

  // Rango del acelerómetro y giroscopio
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // CALIBRACION
  float suma = 0;
  int ciclos = 500;
  for (int t = 0; t < ciclos; t++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    suma = suma + g.gyro.x;
    delay(10);
  }
  sesgo_x = suma / ciclos; 
  delay(10);
}

// FUNCION DE ENVÍO A MATLAB
void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5) {
  Serial.write("abcd"); // Header
  byte *b = (byte *)&dato1;
  Serial.write(b, 4);
  b = (byte *)&dato2;
  Serial.write(b, 4);
  b = (byte *)&dato3;
  Serial.write(b, 4);
  b = (byte *)&dato4;
  Serial.write(b, 4);
  b = (byte *)&dato5;
  Serial.write(b, 4);
}

void loop() {
  unsigned long t_inicial = micros();

  // VARIABLES ESTÁTICAS
  static float alpha_k = 0;
  static float alpha_a = 0;
  static float alpha = 0;
  // IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ESTIMACIÓN DE ÁNGULO (FILTRO COMPLEMENTARIO)
  alpha_k = alpha_k + (g.gyro.x - sesgo_x) * (T_MUESTREO * 1e-6);
  alpha_a = atan2(a.acceleration.y, a.acceleration.z);
  float R = 0.95;
  alpha = (alpha + (g.gyro.x - sesgo_x) * (T_MUESTREO * 1e-6)) * R + alpha_a * (1.0 - R);

  // OBSERVADOR
  float tita = alpha * 180.0 / PI;
  float w = (g.gyro.x - sesgo_x) * 180.0 / PI;

  X_k = {tita,w};

  Matrix<1,1> u_k =(mat_K * X_k)(0,0);

  // Saturación 
  int angulo_servo = constrain(90 - u_k(0,0), 50, 160);

  // Escritura directa en grados
  float angulo_us = 980.0 + (angulo_servo / 180.0) * 1000.0;
  servo.writeMicroseconds((int)angulo_us);
  // Envío de 5 parámetros
  matlab_send(tita,w, u_k(0,0), 0 , 0);


  // CONTROL DE PERIODO DE MUESTREO
  unsigned long t_final = micros();
  unsigned long t_ejecucion = t_final - t_inicial;
  if (t_ejecucion < T_MUESTREO) {
    delayMicroseconds(T_MUESTREO - t_ejecucion);
  }
}