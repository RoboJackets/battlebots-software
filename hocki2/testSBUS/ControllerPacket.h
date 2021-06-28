#ifndef CONTROLLERPACKET_H
#define CONTROLLERPACKET_H

struct ControllerPacket {
	int xSpeed;
	int ySpeed;
	int rotSpeed;
	bool tankDrive;
	bool failsafe;
	bool lostFrame;
};

#endif
