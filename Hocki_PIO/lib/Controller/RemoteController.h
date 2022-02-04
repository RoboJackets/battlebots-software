#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include "sbus.h"
#include "ControllerPacket.h"
#include "Watchdog_t4.h"

class Controller {
	private:
		SbusRx *sbus_rx;
	public:
		Controller();
		WDT_T4<WDT3> wdt;
		void init();
		bool read(ControllerPacket *packet);
};

#endif
