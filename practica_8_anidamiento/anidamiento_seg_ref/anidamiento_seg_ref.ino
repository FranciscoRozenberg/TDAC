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
// Matriz de transición de estados A (2x2)
Matrix<2, 2> mat_A = {  0.9718,  0.01684, 
                       -2.661 ,  0.6921 };

// Vector de entrada B (2 filas x 1 columna)
Matrix<2, 1> mat_B = { 0.01005,
                       0.9477  };

// Vector de salida C (1 fila x 2 columnas: y = C * x)
Matrix<1, 2> mat_C = { 1.0, 0.0 };

// Vector de ganancias del observador L (2 filas x 1 columna)
Matrix<2, 1> mat_L = {  0.4188,
                        1.298 };

// Vector de estado estimado X_hat (2 filas x 1 columna)
Matrix<2, 1> X_hat = { 0.0,
                       0.0 };

Matrix<1, 2> mat_K = { -1.9162  , -0.2581};

Matrix<1,1> F = {4.7235};

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
  static float tita_hat = 0;
  static float w_hat = 0;

  if(contador == 400){
    contador = 0;

    if(ref == 0){
      ref = 7;
    }
    else if(ref == 7){
      ref = 0;
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

  // OBSERVADOR
  float tita = alpha * 180.0 / PI;
  float w = (g.gyro.x - sesgo_x) * 180.0 / PI;
  Matrix<1,1> u_k = ref * F+ (mat_K * X_hat)(0,0);

  // Saturación 
  float angulo_servo = constrain(90.0 - u_k(0,0), 50.0, 160.0);
  servo.write(angulo_servo);

  // tita_hat = C * X_hat (matriz 1x1, se accede con (0, 0))
  Matrix<1, 1> tita_calc = mat_C * X_hat;
  tita_hat = tita_calc(0, 0);
  w_hat=X_hat(1, 0);

  // Envío de 5 parámetros: [giro_x, ángulo_y, x1_hat, x2_hat, y_hat]
  matlab_send(tita, tita_hat,w, w_hat, ref,angulo_servo,90.0 + u_k(0,0),0);

  // Actualización del estado estimado
  //X_hat = (mat_A + mat_B *mat_K)* X_hat + mat_L * (tita - tita_hat) + mat_B *u_k;
  X_hat = mat_A * X_hat + mat_B * u_k(0,0)+ mat_L * (tita - tita_hat);

  // CONTROL DE PERIODO DE MUESTREO
  unsigned long t_final = micros();
  unsigned long t_ejecucion = t_final - t_inicial;
  if (t_ejecucion < T_MUESTREO) {
    delayMicroseconds(T_MUESTREO - t_ejecucion);
  }
}