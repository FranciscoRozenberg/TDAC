#define PIN_POTE A1

unsigned long tiempo_anterior;

unsigned long tiempos[2];
uint8_t V_max_pote = 254;
uint8_t vueltas_pote = 1;

void leer_tiempo_medicion(void);
void leer_tiempo_y_angulo(void);
void lectura_regular(unsigned long T = 20000);

void setup() {
  Serial.begin(2000000);

  pinMode(PIN_POTE, "input");

  tiempo_anterior = micros();


}

void loop() {
//   tiempos[0] = micros();
//   uint8_t medicion_cruda = analogRead(A0);
//   tiempos[1] = micros();

//  // unsigned long tiempo_actual = micros();

//   //Serial.print("Medicion Cruda: ");
//   Serial.print(medicion_cruda);
//   Serial.print(" | ");
//   //Serial.print(" ");
//   Serial.println(tiempos[1] - tiempos[0]);
//   //Serial.println(" ms");
//   //tiempo_anterior = tiempo_actual;

  lectura_regular();

}


void leer_tiempo_medicion(void)
{
  tiempos[0] = micros();
  uint8_t medicion_cruda = analogRead(PIN_POTE);
  tiempos[1] = micros();

  Serial.println(tiempos[1] - tiempos[0]);
}

void leer_tiempo_y_angulo(void)
{
  tiempos[0] = micros();
  uint8_t medicion_cruda = analogRead(PIN_POTE);
  tiempos[1] = micros();

  Serial.print(medicion_cruda /V_max_pote * 360*vueltas_pote);
  Serial.print(" | ");
  Serial.println(tiempos[1] - tiempos[0]);
}

void lectura_regular(unsigned long T = 20000)
{
  tiempos[1] = micros();

  if(tiempos[1] - tiempos[0] >= T)
  {
    Serial.print((tiempos[1] - tiempos[0]));
    
    tiempos[0] = tiempos[1];
    int medicion_cruda = analogRead(PIN_POTE);

    Serial.print(" | ");
    Serial.println(medicion_cruda);
    //Serial.println((tiempos[1] - tiempos[0]));
  }
}