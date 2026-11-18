#include "TimerOne.h"

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // Ajustar condiciones iniciales de trabajo
  static float u0=0.5, h_ref=0.45, h=0.45, u;
  static float Ts=1;
  FLOATUNION_t aux;
  static float sampling_period_ms = 1000*Ts;
  //=========================
  // Definir parametros y variables del control
  static const float b0 = -4.9161f;
  static const float b1 =  4.9059f;
  static const float a1 =  1.0f;

  // Historial
  static float du_prev = 0.0f;
  static float e_prev  = 0.0f;
  float e = 0.0f;
  float du = 0.0f;
  //=========================

  if (Serial.available() >= 8) {
 
    aux.number = getFloat();
    h = aux.number;
    aux.number = getFloat();
    h_ref = aux.number;
  }
  //=========================
  //CONTROL
  e = h_ref - h;

  // Ecuación en diferencias sobre el incremento (delta_u)
  du = (a1 * du_prev) + (b0 * e) + (b1 * e_prev);

  // Sumar punto de operación
  u = u0 + du;
  // Actualizar variables
  e_prev  = e;
  du_prev = du;
  //=========================
    
  matlab_send(u,h_ref,u0);
  delay(sampling_period_ms);
}

void matlab_send(float u, float h, float u0){
  Serial.write("abcd");
  byte * b = (byte *) &u;
  Serial.write(b,4);
  b = (byte *) &h;
  Serial.write(b,4);
  b = (byte *) &u0;
  Serial.write(b,4);
}

float getFloat(){
    int cont = 0;
    FLOATUNION_t f;
    while (cont < 4 ){
        f.bytes[cont] = Serial.read() ;
        cont = cont +1;
    }
    return f.number;
}
