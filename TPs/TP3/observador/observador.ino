#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

// PINES
#define PWM_PIN 5
#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 35
#define PI 3.14159265359

// CREACION DEL SERVO
Servo servo;

// SENSOR DE ULTRASONIDO
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// CREACION DE LA IMU
Adafruit_MPU6050 mpu;

// VARIABLES
const int posicion_inicial = 1500;       // us
const float tiempo_1cm = 29.287;         // us
const unsigned long T_MUESTREO = 20000;  // 20ms = 50 Hz
float sesgo_x = 0;

// MATRICES DEL OBSERVADOR
Matrix<4, 4> mat_A = {
   1.0000,  0.0193,  0.0026,  0.0000,
   0.0000,  0.9316,  0.2542,  0.0023,
   0.0000,  0.0000,  0.9716,  0.0168,
   0.0000,  0.0000, -2.6755,  0.6919
};

Matrix<4, 1> mat_B = {
  0.0000,
  0.0009,
  0.0101,
  0.9476
};

Matrix<2, 4> mat_C = {
  1, 0, 0, 0,
  0, 0, 1, 0
};

Matrix<4, 2> mat_L = {
    0.3399  , -0.3515,
   -1.2001   , 1.4523,
    0.2549 ,   0.4231,
   -2.6413  , -3.0940
};

Matrix<4, 1> X_hat = {0, 0, 0, 0};

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
void matlab_send(float dato1, float dato2, float dato3, float dato4, float dato5, float dato6, float dato7, float dato8) {
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
  b = (byte *)&dato6;
  Serial.write(b, 4);
  b = (byte *)&dato7;
  Serial.write(b, 4);
  b = (byte *)&dato8;
  Serial.write(b, 4);
}

void loop() {
  unsigned long t_inicial = micros();

  // VARIABLES ESTÁTICAS
  static float u = 0;
  static int contador = 0;
  static float alpha_k = 0;
  static float alpha_a = 0;
  static float alpha = 0;
  // MEDICIONES
  static float p = 0;
  static float theta = 0;
  static float w = 0;
  contador++;
  // SERVO
  if (contador == 200) {
    u = -7;
  }
  if (contador == 225) {
    u = 0;
  }
  if (contador == 325) {
    u = 10;
  }
  if (contador == 375) {
    u = 0;
  }
  if (contador == 475) {
    u = -7;
  }
  if (contador == 525) {
    u = 0;
  }
  if (contador == 625) {
    u = 7;
  }
  if (contador == 675) {
    u = 0;
  }
  // ENTRADA DEL SISTEMA
  servo.write(90 - u);

  // IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ESTIMACIÓN DE ÁNGULO (FILTRO COMPLEMENTARIO)
  alpha_k = alpha_k + (g.gyro.x - sesgo_x) * (T_MUESTREO * 1e-6);
  alpha_a = atan2(a.acceleration.y, a.acceleration.z);
  float R = 0.95;
  alpha = (alpha + (g.gyro.x - sesgo_x) * (T_MUESTREO * 1e-6)) * R + alpha_a * (1.0 - R);
  
  theta = alpha * 180.0 / PI;
  w = (float)(g.gyro.x - sesgo_x)*180 /PI;
  
  // ULTRASONIDO
  float p_aux=0;
  unsigned int ping = sonar.ping();
  if (ping == 0) {p_aux = MAX_DISTANCE;
  } else {
    p_aux = (float)ping / 2.0 / tiempo_1cm;
  }
  float d = 19-5.5/2;
  p = d - p_aux;

  // VECTOR DE MEDICIONES
  Matrix<2, 1> Y = {p,theta};

  // OBSERVADOR
  // Salida estimada: Y_hat = C * X_hat
  Matrix<2, 1> Y_hat = mat_C * X_hat;

  //MATLAB
  float p_hat = X_hat(0, 0);
  float dp_hat = X_hat(1, 0);
  float theta_hat = X_hat(2, 0);
  float dtheta_hat = X_hat(3, 0);
  matlab_send(p, p_hat, dp_hat, theta, theta_hat, w, dtheta_hat, u);
  
  // Actualización del estado estimado
  X_hat = mat_A * X_hat + mat_B * u + mat_L * (Y - Y_hat);
  // CONTROL DE PERIODO DE MUESTREO
  unsigned long t_final = micros();
  unsigned long t_ejecucion = t_final - t_inicial;
  if (t_ejecucion < T_MUESTREO) {
    delayMicroseconds(T_MUESTREO - t_ejecucion);
  }
}