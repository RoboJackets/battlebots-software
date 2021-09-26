#ifndef AccelReading_h
#define AccelReading_h

#include "Arduino.h"

class AccelReading
{
  public:
    AccelReading();
    void init(int16_t x, int16_t y, int16_t z, double scalingFactor = 1);
    double accelSize();
    void printDebug();
    double x;
    double y;
    double z;
};

#endif
