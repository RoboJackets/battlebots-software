#ifndef CONTROLLERPACKET_H
#define CONTROLLERPACKET_H

struct ControllerPacket {
	int xSpeed;
	int ySpeed;
	
	float rightud;
	float rightlr;
	float leftud;
	float leftlr;

	int rotSpeed;
	bool tankDrive;
	bool nfs;
	bool failsafe;
	bool lostFrame;
};
/*
 * Error Below:
 */
//bool failsafe;
//bool lostFrame;

#endif
