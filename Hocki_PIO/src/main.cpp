#include "Controller.h"
#include "ControllerPacket.h"
#include "Watchdog_t4.h"

Controller c;
ControllerPacket p;
int i = 0;

void setup() {
	Serial.begin(115200);
	while (!SerialUSB) {}
	c.init();
}

void loop() {
	if (c.read(&p)) {
		Serial.println(p.xSpeed);
		Serial.println(p.ySpeed);
		c.wdt.feed();
	} else {
		Serial.println("Not reading.");
	}
	Serial.println(i);
	delay(10);
	i = i + 1;
}