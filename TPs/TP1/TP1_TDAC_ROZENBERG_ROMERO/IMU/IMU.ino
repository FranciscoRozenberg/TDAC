#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;
const unsigned long T_MUESTREO = 20000;  // 20 ms = 50 Hz
float sesgo_x = 0;

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

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
  int ciclos = 500;
  for (int t = 0; t < ciclos; t++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    suma = suma + g.gyro.x;
    delay(10);
  }
  sesgo_x = suma / ciclos;
  Serial.println("Calibracion terminada");
  Serial.print("Sesgo X: ");
  Serial.println(sesgo_x, 6);
  delay(1000);
}

void loop() {

  const int muestras = 200;
  float media = 0;
  float tiempo_total = 0;
  float M2 = 0;
  static float alpha = 0;
  for (int i = 1; i <= muestras; i++) {
    unsigned long t_inicial = micros();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float alpha_a = atan2(a.acceleration.y,a.acceleration.z);
    alpha = alpha +(g.gyro.x - sesgo_x) *(T_MUESTREO * 1e-6);
    float R = 0.85;
    alpha = alpha * R +alpha_a * (1 - R);
    float delta = alpha - media;
    media = media + delta / i;
    M2 = M2 + delta * (alpha - media);
    unsigned long t_final = micros();
    unsigned long tiempo_medicion =t_final - t_inicial;
    tiempo_total = tiempo_total + tiempo_medicion;

    Serial.print("Angulo: ");
    Serial.print(alpha * 180.0 / PI, 4);
    Serial.print("  Tiempo: ");
    Serial.print(tiempo_medicion);
    Serial.println(" us");
    delay(20);
  }

  float varianza = M2 / muestras;
  float desvio = sqrt(varianza);
  float tiempo_promedio = tiempo_total / muestras;

  Serial.println();
  Serial.print("Promedio angulo: ");
  Serial.print(media * 180.0 / PI, 4);
  Serial.println(" grados");  
  Serial.print("Promedio tiempo: ");
  Serial.print(tiempo_promedio);
  Serial.println(" us");
  Serial.print("Varianza angular: ");
  Serial.print(varianza *(180.0 / PI) *(180.0 / PI), 6);
  Serial.println(" grados^2");
  Serial.print("Desvio estandar: ");
  Serial.print(desvio * 180.0 / PI, 4);
  Serial.println(" grados");
  Serial.println();
  delay(10000);
}