#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "libraries/SBUS/src/SBUS.h"

struct Controller {
	int xSpeed;
	int ySpeed;
	int rotSpeed;
	bool tankDrive;
	bool failsafe;
	bool lostFrame;
}

uint16_t * channels;
bool failsafe;
bool lostFrame;

SBUS receiver(Serial1);

bool ControllerStart();
bool ControllerUpdate(Controller*);

