#include <Arduino.h>
#include "PinDefs.h"
#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"

Controller c(&Serial1);
ControllerPacket p;
void setup() {
  /* Serial to display the received data */
  Serial.begin(115200);
  while (!Serial) {}
}

void loop() {
  c.read(&p);
}