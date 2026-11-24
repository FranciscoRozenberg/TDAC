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
const float Ts = 1.0/50.0;

// MATRICES DE ESTADOS

Matrix<4, 4> mat_A = {
    1.0000,    0.0200,         0,         0,
         0,    0.9291,    0.2660,         0,
         0,         0,    1.0000,    0.0200,
         0,         0,   -3.1780,    0.6678,
};

Matrix<4, 1> mat_B = {
         0,
         0,
         0,
    1.1256,
};


Matrix<2, 4> mat_C = {
  1, 0, 0, 0,
  0, 0, 1, 0
};

Matrix<4, 2> mat_L = {
  0.666352810096982 , -0.095143241799271,
 -0.071049716985826 ,  0.338381652436354,
  0.399441263063421 ,  0.378405200994324,
 -1.958522680588874 , -0.974601939353084,
};

Matrix<4, 1> X_hat = {0, 0, 0, 0};

Matrix<1, 4> mat_K = { -3.5,   -0.2099,    0.8,   -0.0105 };
Matrix<1, 1> H     = { 3 };

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
  static float ref = 0;
  static int contador = 0;
  static float alpha_k = 0;
  static float alpha_a = 0;
  static float alpha = 0;
  // MEDICIONES
  static float p = 0;
  static float theta = 0;
  static float w = 0;
  static float q_k = 0;

if(contador == 400){
    contador = 0;

    if(ref == 0){
      ref = 0;
    }
    else if(ref == 0){
      ref = 0;
    }
    else{
      ref = 50;
    }
  }

  contador++;

  // IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ESTIMACIÓN DE ÁNGULO (FILTRO COMPLEMENTARIO)
  alpha_k = alpha_k + (g.gyro.x - sesgo_x) * (T_MUESTREO * 1e-6);
  alpha_a = atan2(a.acceleration.y, a.acceleration.z);
  float R = 0.95;
  alpha = (alpha + (g.gyro.x - sesgo_x) * (T_MUESTREO * 1e-6)) * R + alpha_a * (1.0 - R);
  // ULTRASONIDO
  float p_aux=0;
  unsigned int ping = sonar.ping();
  if (ping == 0) {p_aux = MAX_DISTANCE;
  } else {
    p_aux = (float)ping / 2.0 / tiempo_1cm;
  }
  float d = 19-5.5/2;
  p = d - p_aux;

  theta = alpha * 180.0 / PI;
  w = (g.gyro.x - sesgo_x) * 180.0 / PI;
  Matrix<2, 1> Y = {p,theta};

  // SALIDA ESTIMADA
  Matrix<2,1> tita_calc = mat_C * X_hat;
  Matrix<2, 1> Y_hat = mat_C * X_hat;
  // ACCION INTEGRAL
  q_k = q_k + Ts * (ref - X_hat(0, 0));
  // LEY DE CONTROL
  Matrix<1,1> u_k = (mat_K * X_hat)(0,0) + H * q_k;
  // SATURACION
  float angulo_servo = constrain(93.0 - u_k(0,0), 50.0, 160.0);
  servo.write(angulo_servo);
  //MATLAB
  float p_hat = X_hat(0, 0);
  float dp_hat = X_hat(1, 0);
  float theta_hat = X_hat(2, 0);
  float w_hat = X_hat(3, 0);
  //matlab_send(p, p_hat, dp_hat, theta, theta_hat, w, w_hat, ref);
  matlab_send(p, p_hat, dp_hat, theta, theta_hat, w, w_hat, u_k(0,0));
  // ACTUALIZACION DEL OBSERVADOR
  X_hat = mat_A * X_hat + mat_B * u_k(0,0) + mat_L * (Y - Y_hat);
  // CONTROL DE PERIODO DE MUESTREO
  unsigned long t_final = micros();
  unsigned long t_ejecucion = t_final - t_inicial;
  delayMicroseconds(T_MUESTREO - t_ejecucion);
}