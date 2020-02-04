/* Mechatronic Creatures
   "Bowl Bot" Genus
   using the Adafruit Huzzah ESP8266 Microcontroller
*/
#include "SHTSensor.h"
#include <DualMAX14870MotorShield.h>
#include <WS2812Serial.h>
#include "Configuration.h"
// audio libraries
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <EEPROM.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// a higher level the more debug printing occurs
#define DEBUG 0
#define TEST_SOLENOIDS 0
#define TEST_MOTOR 0
#define TEST_NEOP 0
// todo make this bring up the stereo test...?
#define TEST_MICS 1
// todo implement this
#define TEST_TEMP_HUMIDITY 0
#define TEST_LDRS 0

#define PRINT_AUDIO_FEATURES 1

// what actuators configurations are present?
#define WOOD_PECKER
#define PECKER_SOL 0

#define BELL_DAMPENER 3
#define BELL_SOL 2
#define BELL_TRENITY

// should the program datalog?
#define DATALOG 1
#define PRINT_EEPROM_ON_BOOT 1

elapsedMillis wb_trigger;
elapsedMillis sol_triggers [6];
unsigned long sol_delays[6] = {200, 1000, 400, 500, 600, 700}; // amount of time inbetween washboard pecker solenoid actuations

//////////////////////////////////////////////////////////////////////////
///////////////////  Debug Printing  /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
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
///////////////////  Solenoids/actuators  ////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define SOL1_PIN 12
#define SOL2_PIN 11
#define SOL3_PIN 7
#define SOL4_PIN 6
#define SOL5_PIN 4
#define SOL6_PIN 3

const int s_pins[] = {SOL1_PIN, SOL2_PIN, SOL3_PIN, SOL4_PIN, SOL5_PIN, SOL6_PIN};
uint16_t sol_on_time[] = {30, 30, 30, 30, 30, 30};
bool sol_state[] = {false, false, false, false, false, false}; // is the solenoid on or off

void testSolenoids(unsigned int len) {
  elapsedMillis t = 0;
  Serial.print("Testing Solenoids - ");
  while (t < len) {
    for (int i = 0; i < 6; i++) {
      Serial.print(i);
      digitalWrite(s_pins[i], HIGH);
      delay(30);
      Serial.print(" ");
      digitalWrite(s_pins[i], LOW);
      delay(100);
    }
    Serial.println();
  }
  Serial.println("Finished testing solenoids");
  Serial.println("--------------------------");
};
//////////////////////////////////////////////////////////////////////////
////////////////////// NeoPixels /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

const uint16_t max_led_count = max(max(LED1_COUNT, LED2_COUNT), LED3_COUNT);

byte drawingMemory[3][max_led_count * 3];       //  3 bytes per LED
DMAMEM byte displayMemory[3][max_led_count * 12]; // 12 bytes per LED

WS2812Serial leds[3] = {WS2812Serial(LED1_COUNT, displayMemory[0], drawingMemory[0], LED1_PIN, WS2812_GRB), 
                        WS2812Serial(LED2_COUNT, displayMemory[1], drawingMemory[1], LED2_PIN, WS2812_GRB), 
                        WS2812Serial(LED3_COUNT, displayMemory[2], drawingMemory[2], LED3_PIN, WS2812_GRB)};

#define BLACK  0x000000
#define RED    0x160000
#define GREEN  0x001600
// changed fro 16 to 46
#define BLUE   0x000046
#define PURPLE 0x160046
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010

//////////////////////////////////////////////////////////////////////////
///////////////////// H-Bridge Motor Driver //////////////////////////////
//////////////////////////////////////////////////////////////////////////
// H-Bridge Motor (MAX14870)
// #define LED_PIN 13
#define MOT_DIR_PIN 21
#define MOT_PWM_PIN 22
#define MOT_EN_PIN 20
#define MOT_FAULT_PIN 2
DualMAX14870MotorShield motor(MOT_DIR_PIN, MOT_PWM_PIN, 31, 32, MOT_EN_PIN, MOT_FAULT_PIN); // 31, 32, and 33 are unused pins

void testMotor(unsigned int len) {
  motor.enableDrivers();
  Serial.println();//"------------------------------");
  Serial.print("Starting Motor Test\n");
  motor.setM1Speed(50);
  Serial.print(" 50\t");
  delay(len / 14);
  motor.setM1Speed(150);
  Serial.print(" 150\t");
  delay(len / 3.5);
  motor.setM1Speed(50);
  Serial.print(" 50\t");
  delay(len / 7);
  // motor.setM1Speed(0);
  // Serial.print(" 0");
  // delay(len / 7);
  motor.setM1Speed(-50);
  Serial.print(" -50\t");
  delay(len / 7);
  motor.setM1Speed(-250);
  Serial.print(" -250\t");
  delay(len / 3.5);
  motor.setM1Speed(-50);
  Serial.print(" -50\t");
  delay(len / 14);
  Serial.print(" 0");
  motor.setM1Speed(0);
  motor.disableDrivers();
  Serial.println("\nFinished Motor Test");
  Serial.println();//"----------------------------");
}

//////////////////////////////////////////////////////////////////////////
/////////////////////  LDR's /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// reading of around 50-60 in the office with normal light
// reading of around 150 when covered
uint8_t ldr_pins[2] = {15, 16};

uint16_t ldr_vals[2];

elapsedMillis last_ldr_reading;

#define LDR_POLL_RATE 30000

void updateLDRs(bool printValues) {
  if (last_ldr_reading > LDR_POLL_RATE) {
    ldr_vals[0] = analogRead(ldr_pins[0]);
    ldr_vals[1] = analogRead(ldr_pins[1]);
    if (printValues) {
      Serial.print("LDR Readings: ");
      Serial.print(ldr_vals[0]);
      Serial.print("\t");
      Serial.println(ldr_vals[1]);
    }
    last_ldr_reading = 0;
  }
}

void testLDRs(uint32_t delay_time) {
  Serial.print("Testing LDR Sensors ");
  calibrateLDRs(delay_time);
}

void calibrateLDRs(uint32_t delay_time) {
  // read temp and humidity 10 times and average the reading over the last 10
  int l1  = 0.0;
  int ll1 = 0.0;
  int l2  = 0.0;
  int ll2 = 0.0;
  delay_time = delay_time / 10;
  Serial.println("\n----- starating ldr calibration -----");
  for (int i = 0; i  < 10; i++) {
    l1 = analogRead(ldr_pins[0]);
    Serial.print("l1: "); Serial.print(l1); Serial.print("\t");
    ll1 += l1;
    l2 = analogRead(ldr_pins[1]);
    ll2 += l2;
    Serial.print("l2: "); Serial.print(l2); Serial.print("\n");
    delay(delay_time);
  }
  ldr_vals[0] = ll1 / 10;
  ldr_vals[1] = ll2 / 10;
  // todo set global brightness values for this
  Serial.print("ldr1_brightness  : ");
  Serial.print(ldr_vals[0]);
  Serial.print("\tldr2_brightness  : ");
  Serial.println(ldr_vals[1]);
  Serial.println("ended LDR calibration");
  Serial.println("-------------------------------");
}

//////////////////////////////////////////////////////////////////////////
////////////////// temp and humidity /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
SHTSensor sht;
double temp = 0.0;
double humidity = 0.0;
elapsedMillis last_shtc_poll;

// the lower the value the less a new reading changes things
#define TEMP_LOWPASS 0.5
#define HUMIDITY_LOWPASS 0.5

#define SHTC_POLLING_RATE 10000

void updateTempHumidity()
{
  // if it is time to update the temp/humidity
  if (last_shtc_poll > SHTC_POLLING_RATE) {
    // poll the reading
    if (sht.readSample()) {
      Serial.print("----------  SHT ---------\n");
      Serial.print("  RH: ");
      double h = sht.getHumidity();
      Serial.print(h, 2);
      Serial.print("\t");
      humidity = (humidity * (1.0 - HUMIDITY_LOWPASS)) + (h* HUMIDITY_LOWPASS);
      Serial.print(humidity, 2);
      Serial.print("\n");
      Serial.print("  T:  ");
      double t = sht.getTemperature();
      Serial.print(t, 2);
      Serial.print("\t");
      temp = (temp * (1.0 - TEMP_LOWPASS)) + (t * TEMP_LOWPASS);
      Serial.print(temp, 2);
      Serial.print("\n");
    } else {
      Serial.print("Error in temp/humidity sensor readSample()\n");
    }
    Serial.println("---------------------------");
    last_shtc_poll = 0;
  }
}

void testTempHumidity(uint32_t delay_time) {
  last_shtc_poll = SHTC_POLLING_RATE + 1;
  updateTempHumidity();
  delay(delay_time);
}

void calibrateTempHumidity(uint32_t delay_time) {
  // read temp and humidity 10 times and average the reading over the last 10
  double h  = 0.0;
  double hh = 0.0;
  double t  = 0.0;
  double tt = 0.0;
  double itters = 0;
  delay_time = delay_time / 10;
  Serial.println("starating temp/humidity calibration");
  for (int i = 0; i  < 10; i++) {
    if (sht.readSample()) {
      Serial.print(itters);
      Serial.print("\t");
      h = sht.getHumidity();
      hh += h;
      Serial.print("h : ");
      Serial.print(h);
      t = sht.getTemperature();
      Serial.print("\tt: ");
      Serial.println(t);
      tt += t;
      itters++;
    }
    delay(delay_time);
  }
  humidity = hh / itters;
  temp = tt / itters;
  Serial.print("humidity set to  : ");
  Serial.println(humidity);
  Serial.print("temp set to      : ");
  Serial.println(temp);
  Serial.println("\nended temp/humidity calibration");
  Serial.println("-------------------------------");
}

//////////////////////////////////////////////////////////////////////////
////////////////// Audio /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

AudioInputI2S            i2s;            //xy=634,246
AudioAmplifier           amp1;           //xy=777.1429023742676,277.14284896850586
AudioAmplifier           input_amp;      //xy=788,240
AudioFilterBiquad        biquad;         //xy=951.4286575317383,241.42859840393066
AudioFilterBiquad        biquad1;        //xy=952.8571243286133,275.71428298950195
AudioAnalyzeRMS          rms;            //xy=1135.4286575317383,322.42859840393066
AudioAnalyzeToneDetect   tone_detect;    //xy=1139.4286575317383,226.42859840393066
AudioAnalyzePeak         peak;           //xy=1139.4286575317383,258.42859840393066
AudioAnalyzeFFT1024      fft1024;        //xy=1145.4286575317383,290.42859840393066
AudioOutputUSB           usb_output;     //xy=1147.4286575317383,194.42859840393066
AudioAnalyzeNoteFrequency note_freq;      //xy=1149.4286575317383,354.42859840393066
AudioConnection          patchCord1(i2s, 0, input_amp, 0);
AudioConnection          patchCord2(i2s, 1, amp1, 0);
AudioConnection          patchCord3(amp1, biquad1);
AudioConnection          patchCord4(input_amp, biquad);
AudioConnection          patchCord5(biquad, tone_detect);
AudioConnection          patchCord6(biquad, peak);
AudioConnection          patchCord7(biquad, fft1024);
AudioConnection          patchCord8(biquad, rms);
AudioConnection          patchCord9(biquad, note_freq);
AudioConnection          patchCord10(biquad, 0, usb_output, 0);
AudioConnection          patchCord11(biquad1, 0, usb_output, 1);

double peak_val = 0.0;
double last_peak = 0.0;

double rms_val;
double last_rms;

double tone_val;
double last_tone;

double freq_val;
double last_freq;

double freq_prob;
double last_freq_prob; // not sure I need this...

int fft_bin_max; //  the bin number which contains the most energy...

void gatherAudioFeatures() {
  // this function will collect and store all the available audio features
  if (peak.available()) {
    last_peak = peak_val;
    peak_val = peak.read() * 10000.0;
    Serial.print("p - ");
    Serial.print(peak_val, 6);
  }
  // rms
  if (rms.available()) {
    last_rms = rms_val;
    rms_val = (double)rms.read() * 10000.0;
    Serial.print("\trms - ");
    Serial.println(rms_val, 6);
  }
  // tone detect
  if (tone_detect.available()) {
    last_tone = tone_val;
    tone_val = tone_detect.read();
    Serial.print("tone - ");
    Serial.println(tone_val);
  }
  // frequency
  if (note_freq.available()) {
    last_freq = freq_val;
    freq_val = note_freq.read();
    last_freq_prob = freq_prob;
    freq_prob = note_freq.probability();
    Serial.print("freq - ");
    Serial.print(freq_val);
    Serial.print(" freq prob: ");
    Serial.println(freq_prob);
  }
  // FFT
  /* TODO
    if (fft.available()) {

    }
  */
  // Serial.println();
}
///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Rhythm detection stuff ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
// the maximum number of rhythms to store
#define MAX_RHYTHMS 10
// the maximum number of notes which can be stored in a single rhythm
#define MAX_NOTES 24

uint8_t active_rhythm; // what rhythm number is to be played next
uint8_t active_notes; // what note number from within the current rhythm will be played next

elapsedMillis last_onset; // when was the last onset detected?
elapsedMillis last_vocalisation; // how long it has been since the bot vocalised last

unsigned long min_inter_note_rhythm = 100; // the shortest amount of time between onsets
unsigned long max_inter_note_rhythm = 1000; // the longest amount of time between onsets

void detectOnset() {
  // given the current audio features determine if an onset has occured
}

/*
  public class Song() {
  // class to store the contents of a song which can be recalled and then played black when deemed suitable
  // contains a list of onsets along with some added information?
  }
  public class Onset() {
  // class to store onsets, should instead be a structure which is contained within a class for rhythms/songs?
  unsigned long when; // when did the onset occur?
  double amp; // how loud was this onset?
  unsigned long dur; // how long did this note last?
  }
*/

void printAudioFeatures() {
  Serial.print("peak: "); Serial.print(peak_val);
  Serial.print("\trms: "); Serial.print(rms_val);
  Serial.print("\ttone: "); Serial.print(tone_val);
  Serial.println();
}

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
  Serial.println("starting setup loop");

  /////////////// solenoid outputs
  pinMode(s_pins[0], OUTPUT);
  pinMode(s_pins[1], OUTPUT);
  pinMode(s_pins[2], OUTPUT);
  pinMode(s_pins[3], OUTPUT);
  pinMode(s_pins[4], OUTPUT);
  pinMode(s_pins[5], OUTPUT);
  digitalWrite(s_pins[0], LOW);
  digitalWrite(s_pins[1], LOW);
  digitalWrite(s_pins[2], LOW);
  digitalWrite(s_pins[3], LOW);
  digitalWrite(s_pins[4], LOW);
  digitalWrite(s_pins[5], LOW);
  Serial.println("Finished setting solenoid pins to outputs");

  /////////////// ldr outputs
  pinMode(ldr_pins[0], INPUT);
  pinMode(ldr_pins[1], INPUT);
  Serial.print("Initial ");
  calibrateLDRs(5000);
  // updateLDRs(true);

  ///////////////////// h-bridge motors
  // motor.enableDrivers();
  motor.flipM1(false);

  ///////////////////// NeoPixel Strips
  leds[0].begin();
  leds[1].begin();
  leds[2].begin();

  ///////////////////// temp and humidity sensor
  Wire.begin();
  if (sht.init()) {
    Serial.print("init(): success\n");
  } else {
    Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
  calibrateTempHumidity(4000);

  ////////////////////// Audio
  AudioMemory(AUDIO_MEMORY);
  // TODO make this proper
  uint32_t lpf = 14000;
  uint32_t hpf = 200;
  double q = 0.8;
  input_amp.gain(1.5);
  biquad.setLowpass(0, lpf, q);
  biquad.setLowpass(1, lpf, q);
  biquad.setHighpass(2, hpf, q);
  biquad.setHighpass(3, hpf, q);
  
  Serial.println("Finished setup Loop");
  delay(8000);
  Serial.println("-----------------------------------");
}

void testNeoStrips(unsigned int len) {
  // todo write this properly without hard coded loop values
  
  Serial.print("Testing NEOP Strips: ");
  Serial.print("1r");
  colorWipe(leds[0], RED, len / 9);
  Serial.print("2r ");
  colorWipe(leds[1], RED, len / 9);
  Serial.print("3r ");
  colorWipe(leds[2], RED, len / 9);
  Serial.print("1o ");
  colorWipe(leds[0], ORANGE, len / 9);
  Serial.print("2o ");
  colorWipe(leds[1], ORANGE, len / 9);
  Serial.print("3o ");
  colorWipe(leds[2], ORANGE, len / 9);
  Serial.print("1p ");
  colorWipe(leds[0], PURPLE, len / 9);
  Serial.print("2p ");
  colorWipe(leds[1], PURPLE, len / 9);
  Serial.print("3p ");
  colorWipe(leds[2], PURPLE, len / 9);
  
  colorWipe(leds[0], BLACK, 0);
  colorWipe(leds[1], BLACK, 0);
  colorWipe(leds[2], BLACK, 0);
  Serial.println("\nFinished testing NeoPixel Strips");
}

void updateAll() {
  // this function keeps all the needed update functions in one place
  updateDatalog();
  updateLDRs(true);
  updateTempHumidity();
  updateSolenoids(); // turns off all solenoids which have been activated using triggerSolenoid
  updateHBridge();   //
  gatherAudioFeatures();
  if (PRINT_AUDIO_FEATURES) {
    printAudioFeatures();
  }
  if (TEST_MOTOR) {
    testMotor(5000);
  }
  if (TEST_SOLENOIDS) {
    testSolenoids(2000);
  }
  if (TEST_NEOP) {
    testNeoStrips(2000);
  } 
  if (TEST_MICS) {
    Serial.println("TODO");
  } 
  if (TEST_TEMP_HUMIDITY) {
    testTempHumidity(500);
  }
  if (TEST_LDRS) {
    testLDRs(1000);
  }
}

void colorWipe(WS2812Serial &l, int color, int wait) {
  for (int i = 0; i < l.numPixels(); i++) {
    l.setPixel(i, color);
    l.show();
    delayMicroseconds(wait);
  }
}

void loop() {
  updateAll();
  // strikeBell();
  // delay(2000);
  /*
    if (sol1_trigger > sol1_delay) {
    triggerSolenoid(0, 30);
    sol1_trigger = 0;
    sol1_delay -= 10;
    if (sol1_delay < 50) {
      sol1_delay = 200;
    }
    }*/
  /*
    if (sol2_trigger > sol2_delay) {
    triggerSolenoid(3, 50);
    sol2_trigger = 0;
    // sol2_delay += 10;
    // if (sol2_delay > 200) {
    //   sol2_delay = 50;
    // }
    }
  */
  /*
    if (wb_trigger > 200) {
    wb_trigger = 0;
    shakeWashboard(50);
    }
  */
}
