#include <TimerOne.h>

volatile float phase = 0;
const float frequency = 10; // Lower frequency for better visibility
const float sampleRate = 500; // 500Hz sample rate
const float phaseIncrement = (2 * PI * frequency) / sampleRate;

void setup() {
  Serial.begin(115200); // Match this in the Plotter dropdown
  while (!Serial);
  
  // Optional: Print a header once for labeling (Plotter accepts this ONLY at start)
  // Serial.println("SineWave:"); 
  
  Timer1.initialize(2000); // 2000 microseconds = 0.002s (500Hz)
  Timer1.attachInterrupt(generateSine);
}

void loop() {
  // Do NOT print anything here while plotting
}

void generateSine() {
  float value = sin(phase);
  phase += phaseIncrement;
  
  if (phase >= 2 * PI) {
    phase -= 2 * PI;
  }
  
  // CRITICAL: Only print the number and the newline. No text.
  Serial.println(value); 
}   