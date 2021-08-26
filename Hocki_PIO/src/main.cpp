#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"

Controller c;
ControllerPacket p;
DriveTrain drive(ESC_L, ESC_R);

int i = 0;

void setup() {
	Serial.begin(115200);
	while (!SerialUSB) {}
	drive.init();
	drive.arm();
	c.init();
}

void loop() {
	if (c.read(&p)) {
		c.wdt.feed();
		if(p.tankDrive){
			SerialUSB.print("X Speed: ");
			SerialUSB.print(p.xSpeed);
			SerialUSB.print("  Y Speed: ");
			SerialUSB.print(p.ySpeed);
			SerialUSB.print("  Rot Speed: ");
			SerialUSB.println(p.rotSpeed);
			int powerL = map(p.xSpeed, 245, 1805, 300, 700);
			int powerR = map(p.xSpeed, 245, 1805, 300, 700);
			drive.setPower(powerL, powerR);
		}
	} else {
		Serial.println("Not reading.");
	}
	Serial.println(i);
	delay(10);
	i = i + 1;
}