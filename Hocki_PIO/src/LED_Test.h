#ifndef PROGRAMS_H
#define PROGRAMS_H

#include "RemoteController.h"
#include "ControllerPacket.h"
#include "ADXL375.h"
#include "DriveTrain.h"
#include "PinDefs.h"
#include "Hocki.h"
#include <FastLED.h>

#define LED_PIN 4
#define CLOCK_PIN 3
#define BUTTON_PIN 5 
#define COLOR_ORDER GBR

#define NUM_LEDS 8
CRGB leds[NUM_LEDS];      //naming our LED array
int brightness = 75;



void setup();
void loop();


void setup() 
{
    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    pinMode(CS3, OUTPUT);
    pinMode(CS4, OUTPUT);

    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
    digitalWrite(CS3, HIGH);
    digitalWrite(CS4, HIGH);

    Serial.begin(115200);
    FastLED.addLeds<DOTSTAR, LEDD1, LEDC1, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  brightness );
}

void loop()
{
    fill_solid(leds, NUM_LEDS, CHSV(128, 128, brightness));  
    FastLED.show();
    delay(500);
    fill_solid(leds, NUM_LEDS, CHSV(200, 128, brightness));  
    FastLED.show();
    delay(500);
}
#endif