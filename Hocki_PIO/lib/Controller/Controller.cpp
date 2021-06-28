#include "ControllerPacket.h"
#include "sbus.h"
#include "Controller.h"
#include "Arduino.h"
#include <array>
Controller::Controller(){
  sbus_rx = new SbusRx(&Serial1);
  startedReading = false;
}

void Controller::init() {
  sbus_rx->Begin();
}

bool Controller::read(ControllerPacket *packet) {
  bool reading = sbus_rx->Read();
  if (reading) {
    std::array<uint16_t, 16> rx_channels = sbus_rx->rx_channels();
      //int rightLR = rx.rx_channels()[0];
      int rightUD = rx_channels[1];
      int leftLR = rx_channels[2];
      //int leftUD = rx.rx_channels()[3];
      bool arcadeMode = (bool) rx_channels[6];
      if (arcadeMode) {
          packet->xSpeed = rightUD;
          packet->rotSpeed = leftLR;
      }
  } 
  return reading;
}
