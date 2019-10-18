/* Mechatronic Creatures
   "Bowl Bot" Genus
   using the Adafruit Huzzah ESP8266 Microcontroller


*/
#include "SparkFun_SHTC3.h" 
#include <DualMAX14870MotorShield.h>
#include <WS2812Serial.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels? 8 for old board
// #define LED_PIN 8
// new design will use pin 5
// #define LED_PIN 5

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 5

// a higher level the more debug printing occurs
#define DEBUG 1

// Solenoids/actuators
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

////////////////////// NeoPixels ////////////////////////////////////////
/*
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
*/
///////////////////// Solenoids //////////////////////////////////////////
const int s_pins[] = {2, 3, 4, 5};

// H-Bridge Motor (MAX14870)
// #define LED_PIN 13
uint8_t mot_dir = 20;
uint8_t mot_pwm = 15;
uint8_t mot_en = 21;
DualMAX14870MotorShield motor(mot_dir, mot_pwm, 31, 32, mot_en, 33); // 31, 32, and 33 are unused pins

// pots
uint8_t pot1 = 16;
uint8_t pot2 = 17;
uint16_t pot1_val = 0;
uint16_t pot2_val = 0;

// jumper button
uint8_t jump_but = 6;

// LDR ??!!??!!
// uint8_t ldr1 = x;
// uint8_t ldr2 = x;
// uint16_t ldr1_val = 0;
// uint16_t ldr2_val = 0;

////////////////// temp and humidity /////////////////////////////
SHTC3 mySHTC3;
double temp = 0.0;
double humidity = 0.0;

////////////////// setup / main loops ////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(1000);// let the system settle
  // strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  // strip.show();            // Turn OFF all pixels ASAP
  // strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  pinMode(s_pins[0], OUTPUT);
  pinMode(s_pins[1], OUTPUT);
  pinMode(s_pins[2], OUTPUT);
  pinMode(s_pins[3], OUTPUT);
  
  ///////////////////// temp and humidity sensor //////////////////////////
  errorDecoder(mySHTC3.begin());  
  Wire.setClock(400000); 
  if(mySHTC3.passIDcrc)                                       // Whenever data is received the associated checksum is calculated and verified so you can be sure the data is true
  {                                                           // The checksum pass indicators are: passIDcrc, passRHcrc, and passTcrc for the ID, RH, and T readings respectively
    Serial.print("ID Passed Checksum. ");
    Serial.print("Device ID: 0b"); 
    Serial.print(mySHTC3.ID, BIN);                       // The 16-bit device ID can be accessed as a member variable of the object
  }
  else
  {
    Serial.print("ID Checksum Failed. ");
  }
  
  //motor.flipM1(true); // if the direction of the motor needs to be flipped
}

elapsedMillis wb_trigger;

void loop() {
  // testPeckerModule();
  
  if (wb_trigger > 2000) {
    wb_trigger = 0;
    shakeWashboard(250);
  }
  updateWashboard(); 
}

/////////////////////////////////////////////////////////
/////////////// SHTC3 Utility Functions /////////////////
/////////////////////////////////////////////////////////
void printSHTCInfo()
{
  if(mySHTC3.lastStatus == SHTC3_Status_Nominal)              // You can also assess the status of the last command by checking the ".lastStatus" member of the object
  {
    humidity = mySHTC3.toPercent();
    Serial.print("RH = "); 
    Serial.print(humidity);                   // "toPercent" returns the percent humidity as a floating point number
    Serial.print("% (checksum: "); 
    if(mySHTC3.passRHcrc)                                     // Like "passIDcrc" this is true when the RH value is valid from the sensor (but not necessarily up-to-date in terms of time)
    {
      Serial.print("pass");
    }
    else
    {
      Serial.print("fail");
    }
    Serial.print("), T = "); 
    temp = mySHTC3.toDegF();
    Serial.print(temp);                        // "toDegF" and "toDegC" return the temperature as a flaoting point number in deg F and deg C respectively 
    Serial.print(" deg F (checksum: "); 
    if(mySHTC3.passTcrc)                                        // Like "passIDcrc" this is true when the T value is valid from the sensor (but not necessarily up-to-date in terms of time)
    {
      Serial.print("pass");
    }
    else
    {
      Serial.print("fail");
    }
    Serial.println(")");
  }
  else
  {
    Serial.print("Update failed, error: "); 
    errorDecoder(mySHTC3.lastStatus);
    Serial.println();
  }
}

void errorDecoder(SHTC3_Status_TypeDef message)                             // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
{
  switch(message)
  {
    case SHTC3_Status_Nominal : Serial.print("Nominal"); break;
    case SHTC3_Status_Error : Serial.print("Error"); break;
    case SHTC3_Status_CRC_Fail : Serial.print("CRC Fail"); break;
    default : Serial.print("Unknown return code"); break;
  }
}
