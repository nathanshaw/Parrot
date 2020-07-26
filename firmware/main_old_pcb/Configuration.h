#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

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

// 
#define SOL1_PIN 12
#define SOL2_PIN 11
#define SOL3_PIN 7
#define SOL4_PIN 6
#define SOL5_PIN 4
#define SOL6_PIN 3

// HBridge Active? if set to false then the HBridge objects just wont be created

#endif
