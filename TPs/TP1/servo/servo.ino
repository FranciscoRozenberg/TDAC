#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define PWM_PIN 5

const unsigned long T_MUESTREO = 20000;  // 20 ms = 50 Hz
Servo servo;
Adafruit_MPU6050 mpu;
const int POSE_DE_SETUP = 1500;  // us
const int POSICION_INICIAL = 2200;  // us
const int POSICION_FINAL = 1100;    // us
float sesgo_x = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  servo.attach(PWM_PIN);
  servo.writeMicroseconds(POSE_DE_SETUP);
  if (!mpu.begin()) {
    Serial.println("NO SE ENCONTRO EL MPU6050");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  float suma = 0;
  const int ciclos = 500;
  for (int i = 0; i < ciclos; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    suma = suma + g.gyro.x;
    delay(10);
  }
  sesgo_x = suma / ciclos;
  delay(2000);
}

void loop() {

  //mınima precision angular
  
  unsigned long t_inicial = micros();
  static int T_BARRA = 1500;  //RECTO   
  static int contador = 0;   
  contador++;   
  if (contador == 50) {
    contador = 0;     
    if (T_BARRA == 1500) {       
      T_BARRA = 1520;     
     } else {T_BARRA = 1500;}   
  }   
  servo.writeMicroseconds(T_BARRA);
  unsigned long t_final = micros();   
  delay((T_MUESTREO - (t_final - t_inicial)) / 1000);

  // diferencial de 30 grados

  // servo.writeMicroseconds(POSICION_INICIAL);
  // delay(1000);
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  // float angulo_inicial =atan2(a.acceleration.y, a.acceleration.z)* 180.0 / PI;
  
  // Serial.print("Angulo inicial: ");
  // Serial.print(angulo_inicial, 3);
  // Serial.println(" grados");

  // unsigned long t_inicial = 0;
  // for (int i = 0; i < 100; i++) {
  //   if (i == 0) {
  //     t_inicial = millis();
  //     servo.writeMicroseconds(POSICION_FINAL);
  //   } else {
  //     mpu.getEvent(&a, &g, &temp);
  //     float angulo_final =atan2(a.acceleration.y, a.acceleration.z)* 180.0 / PI;
  //     if (abs(angulo_final - angulo_inicial) > 30) {
  //       unsigned long t_final = millis();
  //       Serial.print("Angulo final: ");
  //       Serial.print(angulo_final, 3);
  //       Serial.println(" grados");
  //       Serial.print("Tiempo para mover 30 grados: ");
  //       Serial.print(t_final - t_inicial-1);//RESTO 1 SON LOS DELAYS
  //       Serial.println(" ms");
  //       break;
  //     }
  //     delay(1);
  //   }
  // }
  //delay(10000);
}
