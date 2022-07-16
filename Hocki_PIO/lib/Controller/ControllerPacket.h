#ifndef CONTROLLERPACKET_H
#define CONTROLLERPACKET_H

struct ControllerPacket
{
	int xSpeed;
	int ySpeed;
	int rotSpeed;
	bool arcadeDrive;
	bool calibrate;
	bool reversed;
	bool failsafe;
	bool lostFrame;
};
/*
 * Error Below:
 */
// bool failsafe;
// bool lostFrame;

#endif
