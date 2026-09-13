void setup() {

  Serial.begin(115200);
}

void loop() {

  float dato = 1.2;
  unsigned long t_inicial = micros();

  for (int i = 0; i < 100; i++) {
    Serial.write((byte*)&dato, 4);
  }

  unsigned long t_final = micros();
  Serial.println();
  Serial.print("Tiempo de transmision: ");
  Serial.print(t_final - t_inicial);
  Serial.println(" us");
  delay(1000);
}