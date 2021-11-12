#include <Arduino.h>

#define mag_filter_a 38
#define mag_filter_b 39
int va, vb = 0;

void setup() {
  pinMode(mag_filter_a, INPUT);
  pinMode(mag_filter_b, INPUT);
  Serial.begin(9600);
}

void loop() {
  va = analogRead(mag_filter_a);
  vb = analogRead(mag_filter_b);
  Serial.println(va);
  Serial.print(" ");
  Serial.println(vb);
}