#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include "Configuration_adv.h"

#define MAX_BRIGHTNESS        255

// Which pin on the Arduino is connected to the NeoPixels? 8 for old board
#define LED1_PIN 5
// note that if the second LED channel is used the teensy needs to be overclocked to 120 MHz
#define LED2_PIN 8
#define LED3_PIN 10

// How many NeoPixels are attached to the Arduino?
#define LED1_COUNT 16
#define LED2_COUNT 16
#define LED3_COUNT 16

// this is how long in MS to wait after the dampener is removed before striking with solenoid
#define DAMPENER_DELAY 10

#if PCB_REVISION == 0
#define SOL1_PIN 12
#define SOL2_PIN 11
#define SOL3_PIN 7
#define SOL4_PIN 6
#define SOL5_PIN 4
#define SOL6_PIN 3
#define NUM_SOLENOIDS 6
#define NUM_MOTORS    1

#define POT_PIN  14
#define NUM_POTS 1


#elif PCB_REVISION ==   1
// solenoid outputs
#define SOL1_PIN        3
#define SOL2_PIN        4
#define SOL3_PIN        6
#define SOL4_PIN        12
#define SOL5_PIN        11
#define SOL6_PIN        14
#define SOL7_PIN        15
#define SOL8_PIN        16
#define SOL9_PIN        7
#define NUM_SOLENOIDS   9
// motor outputs
#define NUM_MOTORS      3

#define M1_COAST        28
#define M1_SPEED        25
#define M1_DIR          26
#define M1_FAULT        27

#define M2_COAST        31
#define M2_SPEED        32
#define M2_DIR          29
#define M2_FAULT        30

#define M3_COAST        2
#define M3_SPEED        20
#define M3_DIR          21
#define M3_FAULT        22

#define BUT1            33
#define BUT2            A13
#define BUT3            A12

#define POT1            A11
#define POT2            A10

#endif /// PCB_REVISION


///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Status LED and Pot Pin ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
#define LED_PIN         17
#define POT_PIN         14

// HBridge Active? if set to false then the HBridge objects just wont be created

#endif
