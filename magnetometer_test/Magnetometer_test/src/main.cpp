#include <Arduino.h>

#define mag_filter_a 38
#define mag_filter_b 39
float va, vb = 0;

void setup() {
  pinMode(mag_filter_a, INPUT);
  pinMode(mag_filter_b, INPUT);
  Serial.begin(9600);
}

void loop() {
  va = analogRead(mag_filter_a) / 1023.0 * 3.24;
  va = 0;
  vb = analogRead(mag_filter_b) / 1023.0 * 3.24;
  Serial.print(va);
  Serial.print(" ");
  Serial.println(vb);
}