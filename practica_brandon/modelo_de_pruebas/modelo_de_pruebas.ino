#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
//PINES
#define PWM_PIN 5
#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 35
#define PI 3.14159265358979323846
//CREACION DEL SERVO
Servo servo;
//SENSOR DE ULTRASONIDO
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
//CREACION DE LA IMU
Adafruit_MPU6050 mpu;
//VARIABLES
const int posicion_inicial = 1500; //us
const float tiempo_1cm = 29.287;  //us
const unsigned long T_MUESTREO = 20000;  //20ms=50 Hz
float sesgo_x = 0;
//SETUP
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  //SERVO 
  servo.attach(PWM_PIN);
  servo.writeMicroseconds(posicion_inicial); // Posición inicial
  //IMU
  if (!mpu.begin()) {
    Serial.println("NO SE ENCONTRO EL MPU6050");
    while (1) {
      delay(10);
    }
  }
  //Rango del acelerómetro
  //MPU6050_RANGE_2_G MPU6050_RANGE_4_G..... MPU6050_RANGE_16_G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //Rango del giroscopio
  //MPU6050_RANGE_250_DEG MPU6050_RANGE_500_DEG MPU6050_RANGE_1000_DEG MPU6050_RANGE_2000_DEG
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Ancho de banda del filtro
  //MPU6050_BAND_260_HZ MPU6050_BAND_184_HZ MPU6050_BAND_94_HZ MPU6050_BAND_44_HZ MPU6050_BAND_21_HZ MPU6050_BAND_10_HZ MPU6050_BAND_5_HZ
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  //CALIBRACION
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
//FUNCION DE ENVÍO A MATLAB
void matlab_send(float dato1,float dato2,float dato3,float dato4,float dato5) {
  Serial.write("abcd"); //header
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
  int t_inicial = micros();
  //////////////////////////////////////////////////////////////////
  //INICIA EL LOPP
  static int DELTA_ANGULO = 0;  //us RECTO
  static int contador = 0;
  static float alpha_k = 0;
  static float alpha_a = 0;
  static float alpha = 0;
  contador++;
  //SERVO


  // if (contador == 40) {
  //   contador = 0;
  //   if (DELTA_ANGULO == 7) {
  //     DELTA_ANGULO = -7;
  //   } else {
  //     DELTA_ANGULO = 7;
  //   }
  // }

  if (contador == 200) {
    DELTA_ANGULO = -7;
  }
  if (contador == 225) {
    DELTA_ANGULO = 0;
  }
  if (contador == 325) {
    DELTA_ANGULO = 11;
  }
  if (contador == 375) {
    DELTA_ANGULO = 0;
  }
  if (contador == 475) {
    DELTA_ANGULO = -8;
  }
  if (contador == 525) {
    DELTA_ANGULO = 0;
  }
  if (contador == 625) {
    DELTA_ANGULO = 10;
  }
  if (contador == 675) {
    DELTA_ANGULO = 0;
  }
  servo.write(90-DELTA_ANGULO);
  //IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //ULTRASONIDO
  float distancia = 16.25-(sonar.ping(MAX_DISTANCE) / 2.0 / tiempo_1cm);

  //MATLAB
  //matlab_send(a.acceleration.x,a.acceleration.y,a.acceleration.z,distancia,(float)T_BARRA);
  //Serial.print(a.acceleration.x);(a.acceleration.y);(a.acceleration.z);(" m/s^2");
  //Serial.print(g.gyro.x);(g.gyro.y);(g.gyro.y);(" rad/s");
  //Serial.print(temp.temperature);(" degC");
  alpha_k = alpha_k + (g.gyro.x-sesgo_x) * (T_MUESTREO * 1e-6);
  alpha_a = atan2(a.acceleration.y, a.acceleration.z);
  float R = 0.95;
  alpha = (alpha + (g.gyro.x-sesgo_x) * (T_MUESTREO * 1e-6)) * R + alpha_a * (1-R);
  matlab_send(alpha_k*180/PI,alpha_a*180/PI,alpha*180/PI,distancia,(float)DELTA_ANGULO);
  //FINALIZA EL LOOP
  //////////////////////////////////////////////////////////////////
  delayMicroseconds(T_MUESTREO-16383);
  int t_final = micros();
  delayMicroseconds(T_MUESTREO-(t_final-t_inicial));
}