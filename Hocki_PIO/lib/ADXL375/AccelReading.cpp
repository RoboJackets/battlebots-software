#include "Arduino.h"
#include "AccelReading.h"

AccelReading::AccelReading()
{
}

AccelReading::AccelReading(int16_t _x, int16_t _y, int16_t _z, double scalingFactor) : 
    x{_x*scalingFactor},
    y{_y*scalingFactor}, 
    z{_z*scalingFactor}
{}

AccelReading::AccelReading(double _x, double _y, double _z, double scalingFactor) : 
    x{_x*scalingFactor},
    y{_y*scalingFactor}, 
    z{_z*scalingFactor}
{}

void AccelReading::init(int16_t _x, int16_t _y, int16_t _z, double scalingFactor)
{
  x = _x * scalingFactor;
  y = _y * scalingFactor;
  z = _z * scalingFactor;
}

double AccelReading::accelSize()
{
  return sqrt(sq(x) + sq(y) + sq(z));
}

void AccelReading::printDebug()
{
  Serial.print(x,6);
  Serial.print("\t");
  Serial.print(y,6);
  Serial.print("\t");
  Serial.print(z,6);
  Serial.print("\t");
  Serial.println(accelSize());
}

