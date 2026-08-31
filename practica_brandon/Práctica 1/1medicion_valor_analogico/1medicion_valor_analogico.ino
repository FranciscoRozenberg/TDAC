unsigned long tiempo_anterior;
unsigned long periodo_deseado_micro=20000; // 20ms

void setup() {
  Serial.begin(9600);
  tiempo_anterior=micros();
}

void loop() {
  


  int valor_leido =analogRead(A0);
  unsigned long tiempo_actual = micros();
  unsigned long periodo_tiempo= tiempo_actual-tiempo_anterior;
  unsigned long periodo_seg= periodo_tiempo /1000000.0;
  float frecuencia =1/periodo_seg;

  Serial.print("valor ADC =");
  Serial.print(valor_leido);

  float angulo=valor_leido*180/1023;
  Serial.print("angulo =");
  Serial.print(angulo);


  tiempo_anterior=tiempo_actual;
}
