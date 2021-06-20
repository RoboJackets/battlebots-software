#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include "sbus.h"
#include "ControllerPacket.h"

class Controller {
    public:
        Controller();
        //Error here:
        //SbusRx sbus_rx;
        SbusRx *sbus_rx;
        void read(ControllerPacket *packet);
};

#endif
