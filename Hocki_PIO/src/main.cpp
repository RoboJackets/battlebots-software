#include <Arduino.h>
#include "PinDefs.h"
#include "Controller.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"

Controller c(&Serial1);
ControllerPacket p;
DriveTrain drive(ESC_L, ESC_R);

void setup() {
  /* Serial to display the received data */
  SerialUSB.begin(115200);
  while (!SerialUSB) {}
  drive.init();
  drive.arm();
}

void loop() {
  c.read(&p);

  if(p.tankDrive){
    SerialUSB.print("X Speed: ");
    SerialUSB.print(p.xSpeed);
    SerialUSB.print("  Y Speed: ");
    SerialUSB.print(p.ySpeed);
    SerialUSB.print("  Rot Speed: ");
    SerialUSB.println(p.rotSpeed);
    drive.setPower(p.xSpeed + p.rotSpeed, p.xSpeed - p.rotSpeed);
  }
}