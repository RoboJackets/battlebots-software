#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include "sbus-arduino/src/sbus.h"
#include "ControllerPacket.h"

class Controller {
    public:
        Controller();
        SbusRx sbus_rx;
        void read(ControllerPacket packet);
};

#endif
