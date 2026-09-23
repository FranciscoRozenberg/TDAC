#define PIN_POTE A1

void setup() {

  Serial.begin(115200);
  pinMode(PIN_POTE, "input");

}

void loop() {

  // munima resolucion en volts
  // Arduino UNO usa un ADC de 10 bits y una referencia de 5 V, tiene:
  // 2^{10}=1024
  // niveles posibles: de 0 a 1023.
  // Por lo tanto, la mínima variacion de tension que puede distinguir el ADC es:
  // DeltaV=5V/1024 => DeltaV = 4.88mV
  // Sin embargo, el piso de ruido es de +/- 10mV
  // DeltaDeg=270/1024 => DeltaDeg = 0,266
  // Piso de ruido es de +/- 0,2
  //
  
  unsigned long t_inicial = micros();
  float lectura = analogRead(A0);
  unsigned long t_final = micros();

  Serial.print("Lectura: ");
  Serial.print(lectura*5/1023,3);
  Serial.print(" V / ");
  Serial.print(lectura*270/1023,3);
  Serial.print(" deg");
  Serial.print("  Tiempo: ");
  Serial.print(t_final - t_inicial);
  Serial.println( "us");
}