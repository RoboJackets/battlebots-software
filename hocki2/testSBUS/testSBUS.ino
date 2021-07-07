#include "Controller.h"
#include "ControllerPacket.h"

Controller c;
ControllerPacket p;

void setup() {
  /* Serial to display the received data */
  Serial.begin(115200);
  while (!SerialUSB) {}
  /* Begin the SBUS communication */
  //c.sbus_rx->Begin();
}

void loop() {
  c.read(&p);
  Serial.println(p.xSpeed);
  Serial.println(p.ySpeed);
  delay(100);
}
