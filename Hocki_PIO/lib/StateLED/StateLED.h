#ifndef StateLED_H
#define StateLED_H

#include "Arduino.h"
#include <FastLED.h>
#include "../../pins.h"
#include "../../hocki.h"
#include "../../src/Eigen/Dense.h"
#include "../../src/Eigen/Eigen337.h"
#include "../../src/HEKF/HEKF.h"

// define states

#define TOP_TANK    0
#define BOTTOM_TANK 1

#define TOP_ON_UNDERSPEED   2
#define TOP_ON_ATSPEED      3
#define TOP_ON_OFF          4
#define TOP_OFF             5

#define BOTTOM_ON_OVERSPEED 6
#define BOTTOM_ON_ATSPEED   7
#define BOTTOM_ON_OFF       8
#define BOTTOM_OFF          9


// define associated colors
    
#define TANK_COLOR          CRGB::Crimson
#define UNDERSPEED_COLOR    CRGB::Yellow
#define ATSPEED_COLOR       CRGB::SpringGreen
#define ON_OFF_COLOR        CRGB::MidnightBlue
#define OFF_COLOR           CRGB::Black


class StateLED {

    public:
        
        uint8_t topState;
        uint8_t bottomState;

        CRGB top[N_LEDS];
        CRGB bottom[N_LEDS];
        
        StateLED();
        ~StateLED();

        void topOn(HEKF* filter, bool tankDrive, float targheading);
        void setTopColor(CRGB color);
        void topOff();

        void bottomOn(HEKF* filter, bool tankDrive, float targheading);
        void setBottomColor(CRGB color);
        void bottomOff();

    private:

        bool aboutEqual(float a, float b);

};



#endif
