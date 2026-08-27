#include <Servo.h>

#define PWM_PIN 9

unsigned long tiempos[2];

Servo servo;
int angulo;


int T_ON  = 1500; //us
//T_PWM = 

void setup() {
servo.attach(PWM_PIN);
}

void loop() {
servo.writeMicroseconds(T_ON);
}
