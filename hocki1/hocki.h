#ifndef HOCKI_H
#define HOCKI_H

// SPI PARAMS
#define SPIRATE 5000000

// HEKF PARAMS
#define HEKFDT 1.25e-4 // 5 update steps per control loop iteration ideally
#define ALPHA  1.3775
#define BETA   62.0224
#define BOTR   0.0953
#define WHLR   0.0381

// LED PARAMS
#define N_LEDS  8
#define LED_BRIGHNESS   100

// CONTROL PARMS (ALL IN SI)
#define MAX_ANG_ACCEL   164.7874
#define TARG_ANG_VEL    370

#endif
