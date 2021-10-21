
#ifndef PROGRAMS_H
#define PROGRAMS_H
#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "util.h"

void setup();
void loop();
int prevPR = 0;
int prevPL = 0;
int prevMode = 0;
int mode = 0;
Controller c;
ControllerPacket p;
DriveTrain drive(ESC_L, ESC_R);

void setup() {
	Serial.begin(115200);
	drive.init();
	drive.arm();
	c.init();
}

void loop() {
	if (c.read(&p)) {
		c.wdt.feed();
		if(p.tankDrive) {
			mode = 0;
            float left = p.leftud;
            float right = p.rightud;
			int powerL = mapFloat(left, -1, 1, 300, 700);
			int powerR = mapFloat(right, -1, 1, 300, 700);
			drive.setPower(powerL, powerR);
			if (prevMode != mode || prevPR != powerR || prevPL != powerL) {
				SerialUSB.println("TANK ");
				SerialUSB.print("left ");
				SerialUSB.println(powerL);
				SerialUSB.print("right");
				SerialUSB.println(powerR);
			}
			prevPL = powerL;
			prevPR = powerR;
		} else if (p.nfs) {
			mode = 1;
            float move = p.leftud;
            float turn = p.rightlr;
            float left = move + turn;
            float right = move - turn;
			int maxv = max(abs(left), abs(right));
			if (maxv > 1.0) {
				left /= maxv;
				right /= maxv;
			}

			int powerL = mapFloat(left, -1, 1, 300, 700);
			int powerR = mapFloat(right, -1, 1, 300, 700);
			drive.setPower(powerL, powerR);
			if (prevMode != mode || prevPR != powerR || prevPL != powerL) {
				SerialUSB.println("NFS ");
				
				SerialUSB.print("left ");
				SerialUSB.println(powerL);
				SerialUSB.print("right");
				SerialUSB.println(powerR);
			}
			prevPL = powerL;
			prevPR = powerR;
        }
	} 
	prevMode = mode;
	delay(10);
}
#endif

