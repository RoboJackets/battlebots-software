#include "ControllerPacket.h"
#include "sbus.h"
#include "RemoteController.h"
#include "Watchdog_t4.h"
#include "Arduino.h"
#include "PinDefs.h"
#include <array>

/*
 * channel 0: Right Stick L/R
 * channel 1: Right Stick U/D
 * channel 2: Left Stick U/D
 * channel 3: Left Stick L/R
 * channel 4: Knob Left
 * channel 5: Knob right
 * channel 6: SwA
 * channel 7: SwB
 * channel 8: SwC
 * channel 9: SwD
 */
Controller::Controller()
{
	sbus_rx = new SbusRx(&SERIAL_SBUS);
}

void watchdogFailureCallback()
{
	digitalWrite(13, LOW);
	Serial.println("Failure.");
}

void Controller::init()
{
	sbus_rx->Begin();
	WDT_timings_t config;
	config.timeout = 50; /* corresponds to 50 ms */
	config.callback = watchdogFailureCallback;
	wdt.begin(config);
}

bool Controller::read(ControllerPacket *packet)
{
	bool reading = sbus_rx->Read();
	if (reading)
	{
		std::array<uint16_t, 16> rx_channels = sbus_rx->rx_channels();
		// int rightLR = rx.rx_channels()[0];
		int rightUD = rx_channels[1];
		int leftUD = rx_channels[2];
		int rightLR = rx_channels[0];
		int leftLR = rx_channels[3];
		packet->arcadeDrive = (bool)(rx_channels[6] > 1000);
		packet->calibrate = (bool)(rx_channels[9] > 1000);
		packet->reversed = (bool)(rx_channels[7] > 1000);
		packet->xSpeed = rightUD;
		packet->ySpeed = rightLR;
		packet->rotSpeed = leftUD;
	}
	return reading;
}
