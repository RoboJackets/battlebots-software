#include "ControllerPacket.h"
#include "sbus.h"
#include "Controller.h"
#include "Arduino.h"

Controller::Controller(){
  sbus_rx = new SbusRx(&Serial1);
}


void Controller::read(ControllerPacket *packet) {
    if (sbus_rx->Read()) {
        //int rightLR = rx.rx_channels()[0];
        int rightUD = sbus_rx->rx_channels()[1];
        int leftLR = sbus_rx->rx_channels()[2];
        //int leftUD = rx.rx_channels()[3];
        bool arcadeMode = (bool) sbus_rx->rx_channels()[6];
        if (arcadeMode) {
            packet->xSpeed = rightUD;
            packet->rotSpeed = leftLR;
        }
    }
}
