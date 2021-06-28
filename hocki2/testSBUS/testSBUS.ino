#include "Controller.h"
#include "ControllerPacket.h"

Controller c;
ControllerPacket p;

void setup() {
  /* Serial to display the received data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  c.sbus_rx->Begin();
}

void loop() {
  if (c.sbus_rx->Read()) {
    for (int i = 0; i < c.sbus_rx->rx_channels().size(); i++) {
      Serial.print(c.sbus_rx->rx_channels()[i]);
      Serial.print("\t");
    }
  }
  c.read(&p);
}
