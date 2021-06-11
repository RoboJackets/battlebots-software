/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

/*
* This example reads an SBUS packet from an SBUS receiver and writes it to an
* SBUS compatible servo. The SBUS out capability (i.e. writing a command to
* the servo) could be generated independently; however, the packet timing
* would need to be controlled by the programmer, the write function simply
* generates an SBUS packet and writes it to the servos. In this case the
* packet timing is handled by the SBUS receiver and waiting for a good packet
* read.
*/

#include "sbus.h"
#include "Controller.h"
#include "ControllerPacket.h"

Controller c;

void setup() {
  /* Serial to display the received data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
 
}

void loop() {
  c.read();
}
