// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    //Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    //Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    //Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    //Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    //Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    //Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    //Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    //Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  //Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    //Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    //Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    //Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    //Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    //Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    //Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    //Serial.println("5 Hz");
    break;
  }

  //Serial.println("");
  delay(100);
}


void matlab_send(float dato1, float dato2, float dato3){
  Serial.write("abcd");	// defino un header para matlab, cuanto mas largo, mayor probabilidad de que este header nunca se confunda con los datos que mando, pero a cambio mando una mayor cantidad de datos
  
  byte * b = (byte *) &dato1;	// se crea un puntero auxiliar a un tipo de dato de 8 bits, y se usa para apuntar a la dirección de cada dato que debo mandar
  Serial.write(b,4);	//mando por el puerto serie el dato que comienza donde apunta "b", y le indico cuantos bytes debo mandar (si mando floats, son 4 bytes, 32 para doubles, 16 para int_16t y uint_16t, etcetera
  
  b = (byte *) &dato2;	// repito para todos los datos que quiera mandar
  Serial.write(b,4);
  
  b = (byte *) &dato3;
  Serial.write(b,4);
}

void loop() {
  int ti = micros();
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  matlab_send(a.acceleration.x,a.acceleration.y, a.acceleration.z);
  int tf = micros();
  // /* Print out the values */
  // //Serial.print("Acceleration X: ");
  // //Serial.print(a.acceleration.x);
  // //Serial.print(", Y: ");
  // //Serial.print(a.acceleration.y);
  // //Serial.print(", Z: ");
  // //Serial.print(a.acceleration.z);
  // //Serial.println(" m/s^2");

  // //Serial.print("Rotation X: ");
  // //Serial.print(g.gyro.x);
  // //Serial.print(", Y: ");
  // //Serial.print(g.gyro.y);
  // //Serial.print(", Z: ");
  // //Serial.print(g.gyro.z);
  // //Serial.println(" rad/s");

  // //Serial.print("Temperature: ");
  // //Serial.print(temp.temperature);
  // //Serial.println(" degC");

  // //Serial.println("");
  delayMicroseconds(20000-(tf-ti));
}
