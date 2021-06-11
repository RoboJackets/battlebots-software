#ifndef CONTROLLERPACKET_H
#define CONTROLLERPACKET_H

#include "sbus-arduino/src/sbus.h"

struct ControllerPacket {
	int xSpeed;
	int ySpeed;
	int rotSpeed;
	bool tankDrive;
	bool failsafe;
	bool lostFrame;
};

bool failsafe;
bool lostFrame;

#endif
