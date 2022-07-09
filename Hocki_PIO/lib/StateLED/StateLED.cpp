#include "StateLED.h"

StateLED::StateLED()
{
    topState = TOP_OFF;
    bottomState = BOTTOM_OFF;

    FastLED.addLeds<1, DOTSTAR, LEDD1, LEDC1, BGR>(top, N_LEDS);
    FastLED.addLeds<1, DOTSTAR, LEDD2, LEDC2, BGR>(bottom, N_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);

    topOff();
    bottomOff();
}

StateLED::~StateLED()
{
    delete[] top;
    delete[] bottom;
}

void StateLED::setTopColor(CRGB color)
{
    for (uint8_t k = 0; k < N_LEDS; k++)
    {
        top[k] = color;
    }
}

void StateLED::setBottomColor(CRGB color)
{
    for (uint8_t k = 0; k < N_LEDS; k++)
    {
        bottom[k] = color;
    }
}

void StateLED::topOff()
{
    if (topState != TOP_OFF)
    {
        topState = TOP_OFF;
        setTopColor(OFF_COLOR);
    }
}

void StateLED::bottomOff()
{
    if (bottomState != BOTTOM_OFF)
    {
        bottomState = BOTTOM_OFF;
        setBotomColor(OFF_COLOR);
    }
}

void StateLED::topOn(HEKF *filter, bool arcadeDrive, float targheading)
{
    uint8_t currState;
    float angvel = filter->x(1, 0);
    float heading = filter->x(0, 0);

    if (arcadeDrive)
    {
        currState = TOP_ARCADE;
    }
    else if (aboutEqual(heading, targheading) && angvel < TARG_ANG_VEL)
    {
        currState = TOP_ON_UNDERSPEED;
    }
    else if (aboutEqual(heading, targheading) && angvel >= TARG_ANG_VEL)
    {
        currState = TOP_ON_ATSPEED;
    }
    else
    {
        currState = TOP_ON_OFF;
    }

    if (currState == topState)
    {
        return;
    }
    topState = currState;
    bottomOff();

    if (topState == TOP_ARCADE)
    {
        setTopColor(ARCADE_COLOR);
    }
    else if (topState == TOP_ON_UNDERSPEED)
    {
        setTopColor(UNDERSPEED_COLOR);
    }
    else if (topState == TOP_ON_ATSPEED)
    {
        setTopColor(ATSPEED_COLOR);
    }
    else if (topState == TOP_ON_OFF)
    {
        setTopColor(ON_OFF_COLOR);
    }
}

void StateLED::bottomOn(HEKF *filter, bool arcadeDrive, float targheading)
{
    uint8_t currState;
    float angvel = filter->x(1, 0);
    float heading = filter->x(0, 0);

    if (arcadeDrive)
    {
        currState = BOTTOM_ARCADE;
    }
    else if (aboutEqual(heading, targheading) && angvel < TARG_ANG_VEL)
    {
        currState = BOTTOM_ON_UNDERSPEED;
    }
    else if (aboutEqual(heading, targheading) && angvel >= TARG_ANG_VEL)
    {
        currState = BOTTOM_ON_ATSPEED;
    }
    else
    {
        currState = BOTTOM_ON_OFF;
    }

    if (currState == bottomState)
    {
        return;
    }
    bottomState = currState;
    topOff();

    if (bottomState == BOTTOM_ARCADE)
    {
        setBottomColor(ARCADE_COLOR);
    }
    else if (bottomState == BOTTOM_ON_UNDERSPEED)
    {
        setBottomColor(UNDERSPEED_COLOR);
    }
    else if (bottomState == BOTTOM_ON_ATSPEED)
    {
        setBottomColor(ATSPEED_COLOR);
    }
    else if (bottomState == BOTTOM_ON_OFF)
    {
        setBottomColor(ON_OFF_COLOR);
    }
}

bool StateLED::aboutEqual(float a, float b)
{
    return cos(a - b) > 0.996;
    // returns true if a and b are within +/- 5 degrees
}
