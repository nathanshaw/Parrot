/* Mechatronic Creatures
   "Bowl Bot" Genus
   using the Adafruit Huzzah ESP8266 Microcontroller
*/
#include "SHTSensor.h"
#include <DualMAX14870MotorShield.h>
#include <WS2812Serial.h>
// audio libraries
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// a higher level the more debug printing occurs
#define DEBUG 1

// what actuators configurations are present?
#define WOOD_PECKER
// #define BELL_TRENITY

//////////////////////////////////////////////////////////////////////////
///////////////////  Solenoids/actuators  ////////////////////////////////
//////////////////////////////////////////////////////////////////////////
uint16_t sol_on_time[] = {30, 30, 30, 30};
bool sol_state[] = {false, false, false, false}; // is the solenoid on or off

void dbPrint(String msg, uint8_t level) {
  if (level <= DEBUG) {
    Serial.print(msg);
  }
}

void dbPrintln(String msg, uint8_t level) {
  if (level <= DEBUG) {
    Serial.println(msg);
  }
}
//////////////////////////////////////////////////////////////////////////
////////////////////// NeoPixels /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Which pin on the Arduino is connected to the NeoPixels? 8 for old board
#define LED_PIN 8
// new design will use pin 5
// #define LED_PIN 5

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 5
byte drawingMemory[LED_COUNT * 3];       //  3 bytes per LED
DMAMEM byte displayMemory[LED_COUNT * 12]; // 12 bytes per LED
WS2812Serial leds(LED_COUNT, displayMemory, drawingMemory, LED_PIN, WS2812_GRB);

#define RED    0x160000
#define GREEN  0x001600
// changed fro 16 to 46
#define BLUE   0x000046
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010

//////////////////////////////////////////////////////////////////////////
///////////////////// Solenoids //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
const int s_pins[] = {2, 3, 4, 5};

// H-Bridge Motor (MAX14870)
// #define LED_PIN 13
uint8_t mot_dir = 20;
uint8_t mot_pwm = 15;
uint8_t mot_en = 21;
DualMAX14870MotorShield motor(mot_dir, mot_pwm, 31, 32, mot_en, 33); // 31, 32, and 33 are unused pins

//////////////////////////////////////////////////////////////////////////
/////////////////////  LDR's /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// reading of around 50-60 in the office with normal light
// reading of around 150 when covered
uint8_t ldr1_pin = 16;
uint8_t ldr2_pin = 17;
uint16_t ldr1_val = 0;
uint16_t ldr2_val = 0;

elapsedMillis last_ldr_reading;
#define LDR_POLL_RATE 30000

void readLDRs(bool printValues) {
  if (last_ldr_reading > LDR_POLL_RATE) {
    ldr1_val = analogRead(ldr1_pin);
    ldr2_val = analogRead(ldr2_pin);
    if (printValues) {
      Serial.print("LDR Readings: ");
      Serial.print(ldr1_val);
      Serial.print("\t");
      Serial.println(ldr2_val);
    }
    last_ldr_reading = 0;
  }
}
//////////////////////////////////////////////////////////////////////////
/////////////////// jumper button ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
uint8_t jump_but = 6;

//////////////////////////////////////////////////////////////////////////
////////////////// temp and humidity /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
SHTSensor sht;
double temp = 0.0;
double humidity = 0.0;
elapsedMillis last_shtc_poll;
#define SHTC_POLLING_RATE 10000

void readTempAndHumidity()
{
  if (last_shtc_poll > SHTC_POLLING_RATE) {
    if (sht.readSample()) {
      Serial.print("SHT:\n");
      Serial.print("  RH: ");
      Serial.print(sht.getHumidity(), 2);
      Serial.print("\n");
      Serial.print("  T:  ");
      Serial.print(sht.getTemperature(), 2);
      Serial.print("\n");
    } else {
      Serial.print("Error in temp/humidity sensor readSample()\n");
    }
    last_shtc_poll = 0;
  }
}

//////////////////////////////////////////////////////////////////////////
////////////////// Audio /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
AudioInputI2S            i2s1;           //xy=70,94.00000095367432
AudioAmplifier           input_amp_left;           //xy=224,88.00000095367432
AudioFilterBiquad        biquad_left;        //xy=386.00000381469727,88.00000095367432
AudioAnalyzeRMS          rms_left;           //xy=570.0000267028809,169.0000057220459
AudioAnalyzeToneDetect   tone_left;          //xy=574.0000305175781,73.00000286102295
AudioAnalyzePeak         peak_left;          //xy=574.0000305175781,105.00000286102295
AudioAnalyzeFFT1024      fft1024_left;      //xy=580.0000305175781,137.00000381469727
AudioAnalyzeNoteFrequency notefreq_left;      //xy=584.0000305175781,201.00000476837158
AudioConnection          patchCord1(i2s1, 0, input_amp_left, 0);
AudioConnection          patchCord2(input_amp_left, biquad_left);
AudioConnection          patchCord5(biquad_left, tone_left);
AudioConnection          patchCord6(biquad_left, peak_left);
AudioConnection          patchCord7(biquad_left, fft1024_left);
AudioConnection          patchCord8(biquad_left, rms_left);
AudioConnection          patchCord9(biquad_left, notefreq_left);

// AudioOutputUSB           usb_output;           //xy=582.0000305175781,41.00000190734863
// AudioConnection          patchCord3(biquad_left, 0, usb_output, 0);
// AudioConnection          patchCord4(biquad_left, 0, usb_output, 1);

#define AUDIO_MEMORY 24

elapsedMillis last_audio_usage_print;
#define AUDIO_USAGE_POLL_RATE 5000

void printAudioUsage() {
  // print the audio usage every second or so
  if (last_audio_usage_print > AUDIO_USAGE_POLL_RATE) {
    Serial.print("memory usage: ");
    Serial.print(AudioMemoryUsageMax());
    Serial.print(" out of ");
    Serial.println(AUDIO_MEMORY);
    last_audio_usage_print = 0;
    AudioMemoryUsageMaxReset();
  }
}

//////////////////////////////////////////////////////////////////////////
////////////////// setup / main loops ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(3000);// let the system settle

  /////////////// solenoid outputs
  pinMode(s_pins[0], OUTPUT);
  pinMode(s_pins[1], OUTPUT);
  pinMode(s_pins[2], OUTPUT);
  pinMode(s_pins[3], OUTPUT);

  /////////////// ldr outputs
  pinMode(ldr1_pin, INPUT);
  pinMode(ldr2_pin, INPUT);
  Serial.print("Initial ");
  readLDRs(true);

  ///////////////////// temp and humidity sensor
  Wire.begin();
  if (sht.init()) {
    Serial.print("init(): success\n");
  } else {
    Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

  ////////////////////// Audio
  AudioMemory(AUDIO_MEMORY);
  //motor.flipM1(true); // if the direction of the motor needs to be flipped
}

elapsedMillis wb_trigger;

void loop() {
  // testPeckerModule();
  readLDRs(true);
  readTempAndHumidity();
  if (wb_trigger > 2000) {
    wb_trigger = 0;
    shakeWashboard(250);
  }
  updateWashboard();
}
