#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include "sbus.h"
#include "ControllerPacket.h"

class Controller {
    private:
      SbusRx *sbus_rx;
      bool startedReading;
    public:
        Controller();
        void read(ControllerPacket *packet);
};

#endif
