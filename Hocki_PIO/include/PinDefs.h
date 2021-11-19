#ifndef PINS_H
#define PINS_H

/*
This file is for defining electrical characteristics such as pin and peripheral mappings and settings.
*/

#define MOSIPIN 11
#define MISOPIN 12
#define SCLKPIN 13

//CS pins for accelerometers SPI
#define CS1     14
#define CS2     15
#define CS3     10
#define CS4     24

/*
//Interrupts not used
#define INT1A   40
#define INT1B   41
#define INT2A   16
#define INT2B   17
#define INT3A   8
#define INT3B   7
#define INT4A   25
#define INT4B   24
*/

#define ESC_L 28
#define ESC_R 29

#define MAGAPIN 38
#define MAGBPIN 39
#define MAGSR   32

#define INT10K  23
#define INT20K  22
#define PEAK10K 20
#define PEAK20K 21
#define REF10K  18
#define REF20K  19
#define IRSEL   37

#define LEDD1   1
#define LEDC1   2
#define LEDD2   3
#define LEDC2   4

#define SERIAL_SBUS Serial1
#endif
