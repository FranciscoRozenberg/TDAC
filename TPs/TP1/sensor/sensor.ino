#include <NewPing.h>

#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 400  //cm

const float tiempo_1cm = 29.287;         //us
const unsigned long T_MUESTREO = 20000;  //20ms=50 Hz

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(115200);
}


// void loop() {
//   //COMPARO EL TIEMPO DE VUELO DE LA MEDICION CON EL TIEMPO DE 
//   //PROCESAMINTO DEL DATO
//   unsigned long ti_med = micros();
//   unsigned int medicion_cruda = sonar.ping(MAX_DISTANCE);
//   unsigned long tf_med = micros();
//   Serial.print("Tiempo medicion: ");
//   Serial.print((tf_med - ti_med) );
//   Serial.println(" us");
//   Serial.print("Distancia: ");
//   Serial.print(medicion_cruda);
//   Serial.println(" us");
//   Serial.println((tf_med - ti_med)-medicion_cruda);
//   delay(1000);
// }

void loop() {

  float suma_tiempo = 0;
  float suma_distancia = 0;
  const int muestras = 150;
  float tiempos[muestras];
  float distancias[muestras];

  for (int i = 0; i < muestras; i++) {
    unsigned int medicion_cruda = sonar.ping(MAX_DISTANCE);
    tiempos[i] = medicion_cruda;
    suma_tiempo = suma_tiempo + medicion_cruda;
    float distancia = medicion_cruda / 2.0 / tiempo_1cm;
    distancias[i] = distancia;
    suma_distancia = suma_distancia + distancia;
    delay(5);
  }

  float promedio_tiempo = suma_tiempo / muestras;
  float promedio_distancia = suma_distancia / muestras;

  
  float suma_var_tiempo = 0;
  float suma_var_distancia = 0;

  for (int i = 0; i < muestras; i++) {
    suma_var_tiempo =suma_var_tiempo + (tiempos[i] - promedio_tiempo) *(tiempos[i] - promedio_tiempo);
    suma_var_distancia =suma_var_distancia + (distancias[i] - promedio_distancia) *(distancias[i] - promedio_distancia);
  }

  float varianza_tiempo = suma_var_tiempo / muestras;
  float varianza_distancia = suma_var_distancia / muestras;

  Serial.print("Promedio tiempo: ");
  Serial.print(promedio_tiempo);
  Serial.println(" us");
  Serial.print("Varianza tiempo: ");
  Serial.print(varianza_tiempo);
  Serial.println(" us^2");
  Serial.print("Promedio distancia: ");
  Serial.print(promedio_distancia);
  Serial.println(" cm");
  Serial.print("Varianza distancia: ");
  Serial.print(varianza_distancia);
  Serial.println(" cm^2");
  Serial.print("Desvio estandar tiempo: ");
  Serial.print(sqrt(varianza_tiempo));
  Serial.println(" us");
  Serial.print("Desvio estandar distancia: ");
  Serial.print(sqrt(varianza_distancia));
  Serial.println(" cm");
  delay(50);
}

