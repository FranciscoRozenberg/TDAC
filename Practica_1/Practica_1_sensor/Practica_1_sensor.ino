#include <NewPing.h>

#define TRIGGER_PIN 10
#define ECHO_PIN 11
#define MAX_DISTANCE 200 //cm

const float tiempo_1cm = 29.287; //us

unsigned int tiempos[2];

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
void lectura_regular(unsigned int T = 20000);

void setup() {
  Serial.begin(2000000);
  tiempos[0] = micros();

}

void loop() {
lectura_regular();

}

void lectura_regular(unsigned int T = 20000)
{
  tiempos[1] = micros();

  if(tiempos[1] - tiempos[0] >= T)
  {
    //Serial.print((tiempos[1] - tiempos[0]));
    tiempos[0] = tiempos[1];
    
    unsigned int medicion_cruda = sonar.ping(MAX_DISTANCE);

    Serial.print(" | ");
    Serial.println(medicion_cruda/2/tiempo_1cm);
    //Serial.println((tiempos[1] - tiempos[0]));
  }
}