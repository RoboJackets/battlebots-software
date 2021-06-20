#include "Controller.h"
#include "ControllerPacket.h"

Controller c;
ControllerPacket p;

void setup() {
  /* Serial to display the received data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
 
}

void loop() {
  c.read(&p);
}
