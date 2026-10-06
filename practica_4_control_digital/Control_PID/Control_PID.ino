#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NewPing.h>

//Ultrasonico
#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 400  //cm

//PINES
#define PWM_PIN 5
#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 200
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

  //VARIABLES DE CONTROL
  static float ref = 16; //cm
  static float u_anterior = 0;
  static float u_anterior2 = 0;
  static float e_anterior = 0;
  static float e_anterior2 = 0;
  static unsigned long tiempo_anterior_inicial = 0;
  static unsigned long t_final = 0;

  unsigned long t_inicial = micros();
  float distancia = sonar.ping(MAX_DISTANCE) / 2.0 / tiempo_1cm;
  float e_actual = ref - distancia;

  // CONTROL PROPORCIONAL
  // float Kp = 2.7; // periodo de oscilacion 1.65 segundos => frecuencia de 0.606 Hz
  // float delta_Uk = Kp * e_actual;

  // CONTROL PI
  // float T = T_MUESTREO * 1e-6;
  // float Kp = 2;
  // float Ki = 1 ;
  // float delta_Uk = u_anterior + (Kp + Ki*T/2) *e_actual + (-Kp + Ki*T/2) *e_anterior;

  // CONTROL PID
  float T = T_MUESTREO * 1e-6;
  float Kp = 2;
  float Ki = 1;
  float Kd = 0.0001;
  float delta_Uk = (u_anterior2 + (Kp + Ki*T/2 + 2*Kd/T) *e_actual + (Ki*T - 4 * Kd/T) *e_anterior + (-Kp + Ki*T/2 + 2*Kd/T) *e_anterior2);

  //Ajuste PID Ziegler-Nichols
  // float T = T_MUESTREO * 1e-6;
  // float T0 = 1.65;
  // float K  = 2.7;
  // float Kp = K*0.6;
  // float Ki = K*1.2/T0;
  // float Kd = 3*K*T0/40;
  // float delta_Uk = (u_anterior2 + (Kp + Ki*T/2 + 2*Kd/T) *e_actual + (Ki*T - 4 * Kd/T) *e_anterior + (-Kp + Ki*T/2 + 2*Kd/T) *e_anterior2);

  float u_k = 90 + delta_Uk;
  u_k = constrain(u_k, 50, 160);
  servo.write((int)u_k);

  matlab_send(distancia,delta_Uk,e_actual,u_k,(T_MUESTREO - (t_final -  tiempo_anterior_inicial)));

  e_anterior2 = e_anterior;
  e_anterior = e_actual;
  u_anterior2 = u_anterior;
  u_anterior = delta_Uk;

  tiempo_anterior_inicial=t_inicial;
  delayMicroseconds(T_MUESTREO-16383);
  t_final = micros();
  delayMicroseconds((T_MUESTREO - (t_final - t_inicial))); //valores menosres a 16k
}