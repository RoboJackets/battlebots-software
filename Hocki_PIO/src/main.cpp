#include "Controller.h"
#include "ControllerPacket.h"
#include "Watchdog_t4.h"

WDT_T4<WDT3> wdt;
Controller c;
ControllerPacket p;
int i = 0;

void failureHandler() {
  Serial.println("Failure.");
}

void setup() {
  Serial.begin(115200);
  while (!SerialUSB) {}
  c.init();
  WDT_timings_t config;
  config.timeout = 50; /* corresponds to 50 ms */
  config.callback = failureHandler;
  wdt.begin(config);
}

void loop() {
  if (c.read(&p)) {
    Serial.println(p.xSpeed);
    Serial.println(p.ySpeed);
    wdt.feed();
  } else {
    Serial.println("Not reading.");
  }
  Serial.println(i);
  
  delay(10);
  i = i + 1;
}