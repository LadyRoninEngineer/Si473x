/***************************************************************************************************
*                         SSSSSS          444    7777777   333333    222222                        *
*                        SS     S  ii    4 44         77  33    33  22    22                       *
*                        SS             4  44         77        33        22                       *
*                         SSSSSS   ii  44444444      77      3333    222222                        *
*                              SS  ii      44        77         33  22                             *
*                        S     SS  ii      44       77    33    33  22                             *
*                         SSSSSS   ii      44       77     333333   22222222                       *
*                                                                                                  *
*                        Si4732THT (Through Hole Technology) Multiband Radio                       *
*                                                                                                  *
*                      Copyright (c) 2023 Nancy Gail Daniels  All Rights Reserved                  * 
*                                 Version 23.0902.0   July 23, 2023                               *
*                               Programmer: Nancy Gail Daniels - AD5EU                             *
*                    Core Si473x Library (PU2CLR) by Ricardo Lima Caratti, Nov 2019                *
*                               Designed for RP2040 Raspberry Pi Pico                              *
* VERSION   NOTES                                                                                  *
* 23.0723:  Code Creation                                                                          *
* 23.0902:  Updates for Version 2 Board set (V2.0 23.0808 Controller & V2 23.0806 Radio)           *
*                                                                                                  *
***************************************************************************************************/
/*  |    |    |    |    |    |    |    |    |  RULER  |    |    |    |    |    |    |    |    |    |
         1         2         3         4         5         6         7         8         9         0
1234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |    |   */
/********************************************* Comments-->| ***************************************/          
/***************************************************************************************************
*                                     HARDWARE (PIN) DEFINITIONS                                   *
***************************************************************************************************/
/**************************************   RP2040 Processor     ************************************/
#define TFT_RST       10                                  // Connect to Arduino RESET pin
#define TFT_CS        11                                  // TFT chip select
#define TFT_DC         9                                  // TFT SPI Data/Command
#define EN1B          A0                                  // Encoder B input (Arduino 10)
#define EN1A          A1                                  // Encoder A input (Arduino 11)
#define E_BUTTON      A3                                  // Encoder switch (Arduino A2)
#define L_BUTTON      A2                                  // Left function button (Arduino A0) 
#define R_BUTTON      24                                  // Right function button (Arduino A01)
#define RESET_PIN     12                                  // Si4735 reset input
/***************************************************************************************************
*                                     SYSTEM OPERATION VARIABLES                                   *
***************************************************************************************************/
#define FM_BAND       0                                   // FM (Frequency Modulation) band mode
#define AM_BAND       1                                   // AM (Amplitude Modulation) band mode
#define SW_BAND       2                                   // SW (Short Wave) band (LSB & USB) mode
#define LW_BAND       3                                   // LW (Long Wave) band mode
/***************************************************************************************************
*                                   EXTRA COLORS (R5G6B5) Fort GUI                                 *
***************************************************************************************************/
#define DKGREEN       0x3506
#define GRAY          0x8410
#define LTGRAY        0xCE79
#define SKYBLUE       0x9E3A
/***************************************************************************************************
*                                   EXTERNAL LIBRARIES AND OBJECTS                                 *
***************************************************************************************************/
#include <Arduino.h>                                      // Arduino Core Library
#include <Wire.h>                                         // i2C Core Library
#include <SPI.h>                                          // SPI Core Library
#include <EEPROM.h>                                       // RP2040 EEPROM Library
#include <Adafruit_GFX.h>                                 // Adafruit Graphics Library for LED                                     
#include <SI4735.h>                                       // Si4735 support library
SI4735 rx;                                                // Radio receiver object instance
#include <Adafruit_ST7789.h>                              // ST7789 TFT display library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS,TFT_DC,-1);  // TFT display instance (-1 = SYS RESET)
#include <Rotary.h>                                       // Rotary encoder library                                    
Rotary encoder = Rotary(EN1A, EN1B);                     // Rotary encoder object instance
/***************************************************************************************************
*                                          FONTS FOR GUI                                           *
***************************************************************************************************/
#include <Fonts/FreeSans9pt7b.h>                          // Small Font (used for Band/Mode)
#include <Fonts/FreeSansBold12pt7b.h>                     // Bold font (used for RDS)
#include <Fonts/FreeSans12pt7b.h>                         // Small Font (used for Band/Mode)
/***************************************************************************************************
*                                          EXTERNAL DATA                                           *
***************************************************************************************************/
#include <patch_ssb_compressed.h>                         // Compressed SSB patch 
const uint16_t size_content = sizeof ssb_patch_content;   // See ssb_patch_content.h
const uint16_t cmd_0x15_size = sizeof cmd_0x15;           // Array where 0x15 command is in the patch
/***************************************************************************************************
*                                   SYSTEM OPERATION VARIABLES                                     *
***************************************************************************************************/
bool devMode=false;                                       // Console output flag for dev debug
char Version[]={"23.0902"};                               // Software version text string
volatile int encoderCount = 0;                           // Rotary encoder1 count: + or - 1
uint8_t currentBandMode = 0;                              // Reciever mode FM(default)/AM/LW/SW
uint8_t currentRecieveMode = 2;                           // Initialize as AM Mono
uint8_t previousRecieveMode=currentRecieveMode;           // Reciever mode to determine update
uint8_t currentSSBMode = 1;                               // Reciever SSB mode 1: LSB  2: USB
uint16_t currentFrequency;                                // Current tuned frequency
uint16_t previousFrequency;                               // Previous tuned frequency for ∆
uint8_t bandwidthIdx = 0;                                 // !!!
uint8_t currentBandIndex=0;                               // Currently selected band (default/0 = FM)
uint8_t currentAntenna=0;                                 //Auto =0, Whip=1, SW=2
bool ssbLoaded = false;                                   // SSB Patch flag - true when loaded
uint8_t rssi = 0;                                         // Radio Signal Strength (Indicator)
uint8_t previousRSSI=0;                                   // Previous RSSI measurement
uint8_t previousSNR=0;                                    // Previous SNR measurement
char *rdsMessage;                                         // RDS message
char lastRDSMessage[]={"INITIALIZE\0"};                   // Last RDS Message
const char *bandwidth[] =                                 // Reciever Bandwidth in kHz
  {"6", "4", "3", "2", "1", "1.8", "2.5"}; 
bool muteState = true;                                    // Mute status: true=unmuted
bool enterFlag=false;                                     
uint8_t FN_State = 0;                                     // Current encoder function
uint8_t selectState = 1;                                  // Current encoder function
uint8_t modeState = 1;                                   // Current encoder switch function
bool menuEnabled=false;                                   // Flag to indicate and active menu
int8_t menuNumber = 0;                                    // Current menuNumber
unsigned long updateTime=0;                               // Time of last update in mS
unsigned long delayTime=500;                              // Time to delay until update  
static uint8_t logLimits[] =                              // limits (log) for RSSI conversion
  {0,1,2,3,4,10,16,22,28,34,44,54,64,74,84,94,127};    
/***************************************************************************************************
*                                    UI Button Structue and Data                                   *
***************************************************************************************************/
typedef struct                                            // Button data structure
{ 
  const char *itemName;                                   // Button menu Text
  uint16_t itemcolor;                                     // Background Color
  uint8_t itemwidth;                                      // Width of Text in pixels
  uint16_t itemxoff;                                       // Offset of button from left hand side
} Buttons;
Buttons buttonmenu[] =                                    // Button menu data declaration
{
  {"START",ST77XX_MAGENTA,58,9},                          // Start-up Menu Bar          
  {"",DKGREEN,45,97},
  {"",DKGREEN,46,177},
  {"",ST77XX_MAGENTA,50,14},                                 
  {"MENU",ST77XX_MAGENTA,50,14},                          // Tune Menu Bar
  {"TUNE-",DKGREEN,45,92},
  {"TUNE+",DKGREEN,46,175},
  {"MUTE",ST77XX_BLUE,58,261},    
  {"MENU",ST77XX_MAGENTA,50,14},                          // Volume Menu Bar
  {"VOL-",DKGREEN,50,92},
  {"VOL+",DKGREEN,50,175},
  {"MUTE",ST77XX_BLUE,58,261},                      
  {"MENU",ST77XX_MAGENTA,50,14},                          // Scan Menu Bar
  {"SCAN-",DKGREEN,45,92},
  {"SCAN+",DKGREEN,46,175},
  {"MUTE",ST77XX_BLUE,58,261},                      
  {"SELECT",ST77XX_MAGENTA,50,5},                        // Pop-up Window Menu     
  {"PREV",DKGREEN,45,97},
  {"NEXT",DKGREEN,46,177},
  {"EXIT",ST77XX_BLUE,58,261}
};
uint8_t buttonItems=5;                                    // Number of Menu Button Sets

/***************************************************************************************************
*                                UI Screen Menu stucture and Data                                  *
***************************************************************************************************/
typedef struct                                            // Menu item description
{
  uint8_t itemId;                                         // Index of current menu mode
  const char *itemName;                                   // Menu mode description
} modeFN;

modeFN modemenu[] =                                       // Encoder structure data
  {
    {0, "FM"},                      
    {1, "AM"},
    {2, "SSB"},
    {3, "AM"}
  };
uint8_t mode_Items=4;                                     // Number of menu items in structure
uint8_t currentMode=0;                                    // Number of menu items in structure
/***************************************************************************************************
*                           Si4735 internal BW filter structure & data                             *
* 0 Auto select proper channel filter (Default)                                                    *    
* 1 Wide (110 kHz) channel filter           2  Narrow (84 kHz) channel filter                      *
* 3 Narrower (60 kHz) channel filter        4  Narrowest (40 kHz) channel filter                   * 
***************************************************************************************************/
typedef struct                                            // Start of data structure
{
  uint8_t BWSelect;                                       // BW channel filter number
  const char *itemName;                                   // BW channel filter descriptor
} BWCF;                                                   // BWCF (BWChannelFilter) declaration
BWCF bwcfSettings[] =                                     // BW Filter structure data
  {
    {0, "Auto"},                          
    {1, "Wide"},
    {2, "85kHz"},
    {3, "60kHz"},
    {4, "40kHz"}
  };
uint8_t bwchSettingitems=5;                               // Number of menu items in structure
uint8_t currentBWCH = 0;                                  // Current BWCH index (auto)
typedef struct
{
  uint8_t idx;                                            // SI473X device bandwidth index
  const char *desc;                                       // bandwidth description
} Bandwidth;

int8_t bwIdxSSB = 4;
const int8_t maxSsbBw = 5;
Bandwidth bandwidthSSB[] = {
    {4, "0.5"},
    {5, "1.0"},
    {0, "1.2"},
    {1, "2.2"},
    {2, "3.0"},
    {3, "4.0"}};

/***************************************************************************************************
*                                       Si4735 frequency steps                                     *
* FM        5 (50kHz), 10 (100kHz) and 100 (1MHz) are valid                                        *    
* AM/SSB    1 (1kHz) to 1000 (1MHz) are valid                                                      * 
***************************************************************************************************/    
typedef struct                                            // Start of data structure
{
  uint16_t step;                                          // Frequency step (in kHz)
  const char *itemName;                                   // Frequency step descriptor
} freqStep;                                               // freqStep declaration
freqStep freqStepData[] =
  {
    {5,"50k "},                                           // FM
    {10,"100k"},                                          // FM
    {100,"1MHz"},                                         // FM
    {1,"1khz"},                                           // AM/SSB
    {5,"5kHz"},                                           // AM/SSB
    {10,"10k "},                                          // AM/SSB 
    {50,"50k "},                                          // AM/SSB
    {100,"100k"},                                         // AM/SSB
    {1000,"1MHz"}                                         // AM/SSB
  };
uint8_t freqStepitems=9;                                  // Number of menu items in structure
uint8_t currentfreqStep = 1;                              // Current frequency step (FM 100kHz)
/***************************************************************************************************
*                                     BAND STRUCTURE AND DATA                                      *
***************************************************************************************************/
typedef struct
{
  const char *bandName;                                   // Band (text) description
  uint8_t bandType;                                       // Band type (FM, AM, LW or SW)
  uint16_t minimumFreq;                                   // Band minimum frequency
  uint16_t maximumFreq;                                   // Band maximum frequency
  uint16_t currentFreq;                                   // Default frequency
  int8_t currentStepIdx;                                  // Index of tabStepAM:  Defeult frequency step (See tabStepAM)
  int8_t bandwidthIdx;                                    // Index of the table bandwidthFM, bandwidthAM or bandwidthSSB;
} Band;
Band band[] = 
{
  {"FM  ", FM_BAND, 8400, 10800, 10210, 10, 0},           // Broadcast FM
  {"AM  ", AM_BAND, 520, 1720, 1190, 10, 4},              // Broadcast AM
  {"160m", SW_BAND, 1800, 3500, 1900, 1, 4},              // 160 meters (Extra/Adv/Gen)
  {"80m ", SW_BAND, 3500, 4500, 3700, 1, 4},              // 80 meters (All) Data 3.5-3.6
  {"60m ", SW_BAND, 4500, 5500, 4850, 5, 4},              // 60 meters 
  {"49m ", SW_BAND, 5600, 6300, 6000, 5, 4},              // 49 meters
  {"41m ", SW_BAND, 6800, 7800, 7047, 1, 4},              // 40 meters
  {"31m ", SW_BAND, 9200, 10000, 9600, 5, 4},             // 31 meters
  {"30m ", SW_BAND, 10000, 11000, 10000, 1, 4},           // 30 meters
  {"25m ", SW_BAND, 11200, 12800, 12789, 5, 4},
  {"22m ", SW_BAND, 13400, 13900, 13600, 5, 4},
  {"20m ", SW_BAND, 14000, 14500, 14047, 1, 4},           // 20 meters
  {"19m ", SW_BAND, 15000, 15900, 15000, 5, 4},
  {"18m ", SW_BAND, 17200, 17900, 17600, 5, 4},
  {"17m ", SW_BAND, 18000, 18300, 18100, 1, 4},           // 17 meters
  {"15m ", SW_BAND, 21000, 21900, 21200, 1, 4},           // 15 mters
  {"12m ", SW_BAND, 24890, 26200, 25000, 1, 4},           // 12 meters
  {"10m ", SW_BAND, 28000, 30000, 28400, 1, 4},           // 10 meters
  {"CB  ", SW_BAND, 26200, 27900, 27500, 1, 4},           // CB (Citizens) band 
  {"LW  ", LW_BAND, 100, 510, 300, 1, 4}                  // Long wave
};                                           
const int lastBand = (sizeof band / sizeof(Band)) - 1;
int bandIdx = 0;                                          // Current table index

typedef struct                                // Menu item description
{
  const char *menuHeader;                       // Menu mode description
  const uint8_t menuHeadOffset;                  // Menu Header Offset
} MENUHEAD; 
MENUHEAD mHeader[] =                                 // Encoder structure data
  {
   {"BAND",50},
   {"RECIEVE MODE",20},
   {"CHANNEL WIDTH",10},
   {"FREQ STEP",35},
   {"AUDIO OPTIONS",15},
   {"DIAGNOSTIC 4732",0}                      
  };
uint8_t mHeadnumItems=6;                           // Number of menu items in structure


/***************************************************************************************************
*                                     I2C Device Identification                                    *
***************************************************************************************************/
typedef struct                                            // I2C Device Data Structure
  {
  uint16_t i2c_chan;                                      // I2C address
  const char *I2CDev;                                     // I2C device name
  }
I2CDev;
I2CDev i2cDev[] = 
  {
  {0x11, "Reciever"},                                     // Si4735
  {0x20, "i2C Exp"},                                      // MCP23008 on bandpass filter
  {0x21, "i2C Exp"},                                      // MCP23008 on LNA attenuator
  {0x26, "i2C Exp"},                                      // MCP23008 on display panel
  {0x36, "Bat Mon"},                                      // MCP23008 on display panel
  {0x42, "UBLOX GPS"},                                    // ublox MAX-10S
  {0x50, "EEPROM"},                                       // TPA2016 i2C Amp
  {0x58, "Audio Amp"},                                    // TPA2016 i2C Amp
  {0x60, "Clk Gen"},                                      // Skyworks Si5351
  {0x63, "Reciever"},                                     // Si4735
  {0x70, "LED Driver"}                                    // Holtek HT16K33
  };
uint8_t Numi2CDev = (sizeof i2cDev / sizeof(I2CDev)) ;    // Calculate number of i2C items in struct
/***************************************************************************************************
*                                                 SET UP                                           *
***************************************************************************************************/
void setup()
{
/***************************************************************************************************
*                                         PROCESSOR PINS SET UP                                    *
***************************************************************************************************/ 
  pinMode(RESET_PIN ,OUTPUT);                             // Set Si4732 reset line to output
  digitalWrite(RESET_PIN, HIGH);                          // Pull Si4732 reset high (not reset)
  pinMode(E_BUTTON ,INPUT_PULLUP);                        // Set encoder switch to input with pullup
  pinMode(L_BUTTON ,INPUT_PULLUP);                        // Set left button to input with pullup
  pinMode(R_BUTTON ,INPUT_PULLUP);                        // Set left button to input with pullup

/***************************************************************************************************
*                                   ROTARY ENCODER INTERRUPT SET UP                                *
***************************************************************************************************/ 
  attachInterrupt(digitalPinToInterrupt(EN1A), rotaryEncoder, CHANGE); // Encoder A
  attachInterrupt(digitalPinToInterrupt(EN1B), rotaryEncoder, CHANGE); // Encoder B

/***************************************************************************************************
*                                       User Interface Startup                                     *
***************************************************************************************************/
  if(devMode)                                             // Check if in development mode
  {
    Serial.begin(9600);                                   // Set console baud rate
    while(!Serial){}                                      // Wait for console availability
  }    
  tft.init(170, 320);                                     // Initialize ST7789 TFT 172x320
  tft.setRotation(3);                                     // Set landscape screen orientation 

  displayStartup();
  //EEPROM.begin(256);                                    // Initialize 256 bytes of EEPROM
  //uint16_t lastFreq;                          
  //EEPROM.get(0,lastFreq);
  //uint8_t lastVolume;
  //EEPROM.get(2,lastVolume);
  //uint8_t lastIndex;
  //EEPROM.get(3,lastIndex);
  int16_t si4735Addr = rx.getDeviceI2CAddress(RESET_PIN); // Check for Si4735
  if ( si4735Addr == 0 ) haltCatchFire();                 // No radio - bad juju many devils
  //si4735Addr=0x63;
  //rx.setDeviceI2CAddress(0x63);
  ///rx.setRefClock(32768);                                  // Set 4735 to external 32768 clock
  //rx.setRefClockPrescaler(1);                             // Additional setup for external clock
  rx.setup(RESET_PIN, 0, 1, SI473X_ANALOG_AUDIO); // Starts FM mode
  rx.setFM(8400, 10800, 10110, 10);                       // FM (84 to 108MHz)/100.3 MHz/Step 100kHz
  rx.setRdsConfig(1, 2, 2, 2, 2);                         // Set up Si4735 for RDS
  delay(200);                                             // Wait for radio to settle
  currentFrequency=previousFrequency=rx.getFrequency();   // Establish starting Frequency
  rx.setVolume(40);                                       // Set inital volume
  rx.setGpioCtl(1, 0, 0);                                 // Setup Si4735 i/o lines
  rx.setGpio(muteState, 0, 0);                            // Set to unmute and whip antenna
  displayRadioScreen();                                   // Display the Radio Screen
}
/***************************************************************************************************
*                                            MAIN PROGRAM                                          *
***************************************************************************************************/
void loop()
{
  if (encoderCount != 0)                                 // Check for a change in tune
  {
    if(!menuEnabled)
    {
      if(modeState==1) changeFrequency();
      if (modeState==2) changeVolume();
      if(modeState==3) seekStation();
    }
  else
    {
      if(encoderCount>0)
      {
        menuNumber++;
        if (menuNumber>=mHeadnumItems) menuNumber=0;
        displayPopUp(menuNumber);
        displayMenu(menuNumber);
      }
      else
      {
      menuNumber--;
        if (menuNumber<=-1) menuNumber=mHeadnumItems-1;
        displayPopUp(menuNumber);
        displayMenu(menuNumber);
      }
    encoderCount=0;
    }
  }
  // Service encoder switch
  if(!digitalRead(E_BUTTON))
    {
    modeState++;
    if (modeState==4) modeState=1;
    displayButtonBar(modeState);
    delay(300);
    } 
// Service right button
   if(!digitalRead(R_BUTTON)) 
    {
      muteState=!muteState;
      tft.setFont(&FreeSans9pt7b);                            // Set to nice font
      tft.setTextSize(1);                                     // Base text size
      tft.setTextColor(ST77XX_WHITE);                         // Use white for text                  
      rx.setGpio(muteState, 0, 0);
      tft.setCursor(buttonmenu[7].itemxoff, 168);  // Set cursor to barSelect(ed) menu bar
      if(muteState) 
      {
        tft.fillRect (242, 153, 78, 17,ST77XX_BLUE);            // Right Button box
        tft.print(buttonmenu[7].itemName);      // Display the menu label 
      }
      else
      {
        tft.fillRect (242, 153, 78, 17,ST77XX_RED);            // Right Button box
        tft.print(buttonmenu[7].itemName);      // Display the menu label 
      }
      resetFont();
      delay(300);
    }
    
  
  if(!digitalRead(L_BUTTON)) 
  {
    if (!menuEnabled)
    {
      menuEnabled=true;
      displayButtonBar(4);
      displayPopUp(menuNumber);
      displayMenu(menuNumber);
    }
    else
    {
      menuEnabled=false;
      tft.fillRect(0,1,172,150,ST77XX_BLACK); // Clear menu
      displayRadioScreen();
    }
  delay(300);
  }
  
  
/***************************************************************************************************
*                           Timer based update of reciever data                                    *
***************************************************************************************************/
/*********************************************************|    ************************************/     
  
  if((millis()-updateTime>=delayTime)&(!menuEnabled))      // If no menu & 500ms (delayTime) has elapsed
  {
    updateTime=millis();                                  // Update to latest

    checkRDS();                                           // Check for a new RDS message}
    displaySNR();
    displayRSSI();                                        // Update signal strength  
    displayRecieveMode();                                 // Check for update to mode

   // displayBattery();                                     // Check current battery status  
  }
 
 }
/*************************************** END OF MAIN PROGRAM **************************************/ 

/***************************************************************************************************
*       setBand(uint8_t bandIndex): set Band                                                       *
***************************************************************************************************/
void setBand (uint8_t bandIndex)
{
  switch (bandIndex)
  {
    case 0:                                               // FM Broadcast
      rx.setFM(band[bandIndex].minimumFreq, band[bandIndex].maximumFreq, band[bandIndex].currentFreq, band[bandIndex].currentStepIdx); 
      rx.setSeekAmRssiThreshold(0);
      rx.setSeekAmSNRThreshold(10);
      rx.setRdsConfig(1, 2, 2, 2, 2);
      rx.setFifoCount(1);
      rx.setGpioCtl(1, 0, 0);                       // Setup Si4735 i/o lines
      rx.setGpio(!muteState, 0, 0);                  // Set to current mute state
      currentBandMode=FM_BAND;
      displayStatus();
      break;
    case 1:
      rx.setAM(band[bandIndex].minimumFreq, band[bandIndex].maximumFreq, band[bandIndex].currentFreq, band[bandIndex].currentStepIdx); 
      rx.setBandwidth(3,1);
      rx.setAvcAmMaxGain(48); // Sets the maximum gain for automatic volume control on AM mode.
      rx.setSeekAmLimits(band[bandIndex].minimumFreq, band[bandIndex].maximumFreq);
      rx.setSeekAmSpacing(10); // spacing 10kHz
      rx.setGpioCtl(1, 0, 0);                       // Setup Si4735 i/o lines
      rx.setGpio(!muteState, 0, 0);                  // Set to current mute state   
      currentBandMode=AM_BAND;
      tft.fillRect(0,25,158,25,ST77XX_BLACK);               // Clear RDS indicator
      displayStatus();
      break;
    case 19:    
      rx.setAM(band[bandIndex].minimumFreq, band[bandIndex].maximumFreq, band[bandIndex].currentFreq, band[bandIndex].currentStepIdx); 
      rx.setAM(520, 1750, 1190, 10);
      rx.setBandwidth(3,1);
      rx.setAvcAmMaxGain(48); // Sets the maximum gain for automatic volume control on AM mode.
      rx.setSeekAmLimits(band[bandIndex].minimumFreq, band[bandIndex].maximumFreq);
      rx.setSeekAmSpacing(10); // spacing 10kHz
      rx.setGpioCtl(1, 0, 0);                       // Setup Si4735 i/o lines
      rx.setGpio(!muteState, 0, 0);                  // Set to current mute state   
      currentBandMode=LW_BAND;
      tft.fillRect(0,25,158,25,ST77XX_BLACK);               // Clear RDS indicator
      displayStatus();
      break;
    default:
      loadSSB();
      rx.setTuneFrequencyAntennaCapacitor(1); // Set antenna tuning capacitor for SW.
    // Last digit is mode - 1 for LSB and 2 for USB
      rx.setSSB(band[bandIndex].minimumFreq, band[bandIndex].maximumFreq, band[bandIndex].currentFreq, band[bandIndex].currentStepIdx,1); 
      rx.setSSBAutomaticVolumeControl(1);
      rx.setSSBSidebandCutoffFilter(0);
      rx.setSSBBfo(50);
      rx.setGpioCtl(1, 0, 0);                       // Setup Si4735 i/o lines
      rx.setGpio(!muteState, 0, 0);                 // Set to current mute state   
      currentBandMode=SW_BAND;   
      tft.fillRect(0,25,158,25,ST77XX_BLACK);               // Clear RDS indicator
      displayStatus();
    break;
  }
  delay(200);
}  



/*******************************************************************************
*                                                                              *
*******************************************************************************/ 
void selectStatus(uint8_t selection)
  {
  uint16_t selectBGColor;
  if(enterFlag)
    {
      selectBGColor=ST77XX_YELLOW;
    }
  else
  {
      selectBGColor=ST77XX_WHITE;
    }
  
  if (selection==1)
    {tft.setTextColor(ST77XX_BLACK,selectBGColor);}
  else  
    {tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);}
  tft.setCursor(65,5);
  tft.print(band[bandIdx].bandName);
  if (selection==2)
    {tft.setTextColor(ST77XX_BLACK,selectBGColor);}
  else  
    {tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);}
  tft.setCursor(65,25);
  tft.print(modemenu[currentMode].itemName);
  if (selection==3)
      {tft.setTextColor(ST77XX_BLACK,selectBGColor);}
  else  
    {tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);}
  tft.setCursor(65,45);
  tft.print(freqStepData[currentfreqStep].itemName);
  if (selection==4)
      {tft.setTextColor(ST77XX_BLACK,selectBGColor);}
  else  
    {tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);}
  tft.setCursor(65,65);
  tft.print(bwcfSettings[currentBWCH].itemName);
  if (selection==5)
      {tft.setTextColor(ST77XX_BLACK,selectBGColor);}
  else  
    {tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);}
  tft.setCursor(65,85);
  tft.print("ON");
  if (selection==6)
      {tft.setTextColor(ST77XX_BLACK,selectBGColor);}
  else  
    {tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);}
  tft.setCursor(65,105);
  tft.print("OFF");
  }


// Show current frequency
void showStatus()
{
  // rx.getStatus();

  previousFrequency = currentFrequency = rx.getFrequency();
  rx.getCurrentReceivedSignalQuality();

  displayFrequency(currentFrequency);
  displayReticle();
}









void drawBandSelect()
{
  tft.setTextSize(2);                       // All text in a 10x14 font
  tft.setTextColor(ST77XX_WHITE);           // Use white for text
  tft.fillRoundRect(1,2,118,120,4, ST77XX_BLACK);
  tft.fillRoundRect(1,2,118,18,4,ST77XX_MAGENTA);
  tft.setCursor(40,4);
  tft.print("BAND");
  //tft.drawRoundRect(0,1,120,122,4,ST77XX_WHITE); // Status Box  
  for(int i=0;i<=5;i++)
  {
  tft.setCursor(5,i*17+23);                      // Positon cursor
  tft.setTextColor(ST77XX_WHITE);           // Use white for text
  if (i==bandIdx) tft.setTextColor(ST77XX_BLACK,ST77XX_GREEN);
  tft.print(band[i].bandName);
  }
}

/***************************************************************************************************
*           displayBandMode(): display current Band and Mode (e.g. Stereo/Mono/USB/LSB)            *
**********************************************************XX*****************************************/
void displayBandMode()
{
  tft.setTextColor(ST77XX_YELLOW);                         // Use green for Band/Mode Text
  tft.setFont(&FreeSans12pt7b);                       // Set to nice font
  tft.setTextSize(1);                                     // Native size
  tft.setCursor(3,21);                                  // Positon cursor
  tft.fillRect(1,2,77,24,ST77XX_BLACK);
  tft.print(band[bandIdx].bandName);                      // Display the message`
  resetFont();
  displayRecieveMode();
}
/***************************************************************************************************
*          displayRecieveMode(): display current recieve mode (USB, LSB, Mono, Stereo)             *
*                       0 for USB, 1 for LSB, 2 for Mono, 3 for Stereo                             *
***************************************************************************************************/
void displayRecieveMode()
{ 
  switch(band[bandIdx].bandType)
  {
    case FM_BAND:
      if (rx.getCurrentPilot()) currentRecieveMode=3;                   // Stereo FM
      else currentRecieveMode=2;                                        // Mono FM
    break;      
    case AM_BAND:
      currentRecieveMode=2;                                             // Mono AM
    break;
    case SW_BAND:
      if (currentSSBMode==1) currentRecieveMode=0;                      //USB
      else currentRecieveMode=1;                                        //LSB
    break;
  }
  if (currentRecieveMode!=previousRecieveMode)
  {
    previousRecieveMode=currentRecieveMode;
    tft.setTextColor(ST77XX_YELLOW);                         // Use green for Band/Mode Text
    tft.setFont(&FreeSans12pt7b);                       // Set to nice font
    tft.setTextSize(1);                                     // Native size
    tft.fillRect(55,2,72,24,ST77XX_BLACK);
    switch (currentRecieveMode)
    {  
      case 0:
        tft.setCursor(79,21);
        tft.print("USB");
      break;
      case 1:
        tft.setCursor(79,21);
        tft.print("LSB"); 
      break;
      case 2:
        tft.setCursor(55,21);                                   // Set cursor for AM/FM Band  
        tft.print("Mono"); 
      break;     
      case 3:
        tft.setCursor(55,21);                                   // Set cursor for AM/FM Band
        tft.print("Stereo");
      break; 
    }        
    resetFont();
  }
}
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!                 Known Good Frozen Code - DO NOT CHANGE (Unless moved above this line)            !
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* beepBuzzer(): Activate buzzer for 50ms                                                           *
* changeVolume(): Service volume encoder change (volume up or down)                                *
* checkRDS(): In FM Mode check for a new RDS message                                               *
* displayBattery(): Display current battery state                                                  *
* displayButtonBar(uint8_t barSelect): place labels on button bar based on barSelect               *             
* displayFrequency(uint32_t frequency): Display current frequency formatted for LED display        *
* displayReticle(): display current frequency range reticle                                        *
* displayRDS(): In FM Mode check for a new RDS message                                             *
* displayRadioScreen(): display the Radio user Screen                                              *
* displayStartup(): Display radio startup screen & HELLO on LED's                                  *
* displayStatus(): update the status box on the TFT                                                *
* displayVolume(): Display Si4735 volume (bar graph) from 0-63                                     *
* getStrength(): convert recieved signal strength to log VU equivalent and return value            *
* haltCatchFire(): If radio chip is not found, display error message & halt                        *
* i2cScan(): Scan each of 127 i2c addresses across for devices                                     *
* loadSSB(): upload SSB patch into the Si4735 or Si4732 recievers                                  *
* rotaryEncoder(): interrupt handler for rotary encoder                                            *
***************************************************************************************************/
/*********************************************************|    ************************************/

/***************************************************************************************************
*             changeVolume(): Service volume change (volume up or down)                            *
***************************************************************************************************/
void changeVolume()
{

  if (encoderCount<0)                                    // Check if volume going down
  {
    rx.volumeDown();                                      // Reduce volume
  }
  else                                                    // Otherwise volume going up
  {
    rx.volumeUp();                                        // Increase volume       
  }
  encoderCount =0;                                       // Reset the encoder count
  displayVolume();                                        // Update new volume on display
}
/***************************************************************************************************
*             changeVolume(): Service volume encoder change (volume up or down)                    *
***************************************************************************************************/
void changeFrequency()
{
  if (encoderCount<0)                                    // Check if volume going down
  {
    rx.frequencyDown();                                    // Reduce volume
  }
  else                                                    // Otherwise volume going up
  {
    rx.frequencyUp();                         // Increase volume       
  }
  encoderCount =0;   
  previousFrequency = currentFrequency = rx.getFrequency();
  displayFrequency(currentFrequency);
  displayReticle();
}
/***************************************************************************************************
*             scanFrequency(): Service volume encoder change (volume up or down)                    *
***************************************************************************************************/
void seekStation()
{
  if (encoderCount<0)                                    // Check if volume going down
  {
    rx.seekStationDown();                                    // Reduce volume
  }
  else                                                    // Otherwise volume going up
  {
    rx.seekStationUp();                         // Increase volume       
  }
  encoderCount =0;   
  previousFrequency = currentFrequency = rx.getFrequency();
  displayFrequency(currentFrequency);
  displayReticle();
}    
/***************************************************************************************************
*             checkRDS(): In FM Mode check for a new RDS message                                   *
***************************************************************************************************/ 
void checkRDS()
{
  if(currentBandMode>=1) return;                          // If not in FM mode (0) return

  rx.getRdsStatus();                                      // Check if RDS is active
  if (rx.getRdsReceived())                                // Check for an RDS message
  {
    if (rx.getRdsSync() && rx.getRdsSyncFound())          // Ensure everything is synced up
    {
      rdsMessage = rx.getRdsText0A();                     // Get the RDS message
      if (rdsMessage != NULL)                             // Ensure there is something there
      {
      if(strcmp(lastRDSMessage,rdsMessage)== 0) return;   // Check if its an old message
      strcpy(lastRDSMessage,rdsMessage);                  // Store new message in old message
      displayRDS();                                       // Display RDS message
      }      
    }
  }
  else                                                    // No RDS Message
  {
    tft.fillRect(0,25,158,25,ST77XX_BLACK);               // Clear RDS indicator
  }

}

/***************************************************************************************************
*                displayButtonBar(uint8_t barSelect): Button Bar                                   *
***************************************************************************************************/
void displayButtonBar(uint8_t barSelect)
{
  tft.fillRect (0, 154, 78, 17,ST77XX_MAGENTA);           // Left Button box
  tft.fillRect (81, 154, 78, 17,DKGREEN);                 // Middle Left Button box
  tft.fillRect (162, 154, 78, 17,DKGREEN);                // Middle Right Button box
  tft.fillRect (242, 154, 78, 17,ST77XX_BLUE);            // Right Button box
  tft.setFont(&FreeSans9pt7b);                            // Set to nice font
  tft.setTextSize(1);                                     // Base text size
  tft.setTextColor(ST77XX_WHITE);                         // Use white for text
  for (int i=0;i<=3;i++)                                  // Cycle through 4 menu labels
  {
    tft.setCursor(buttonmenu[i+(barSelect*4)].itemxoff, 169);  // Set cursor to barSelect(ed) menu bar
    tft.print(buttonmenu[i+(barSelect*4)].itemName);      // Display the menu label 
  }                          
    resetFont();
}
/***************************************************************************************************
*             displayFrequency(): Display current frequency formatted for LED display              *
*             HTK1633 low level write to internal memory set up as 8(digit)x16(segment)            *
***************************************************************************************************/
void displayFrequency(uint32_t frequency)
{
  float tempfreq=frequency;
  uint8_t xoffset=40;
  switch (currentBandMode)                                // Format for each band type
  {
    case FM_BAND:
    tempfreq=tempfreq/100;
    tft.setFont(&FreeSansBold12pt7b);                       // Set to nice font
    tft.setTextSize(2);                                     // Base text size
    if (tempfreq<100) xoffset=68;
    tft.setCursor(xoffset, 140);                                  // Set cursor
    tft.setTextColor(ST77XX_WHITE);                               // In White
    tft.fillRect (40, 100, 130, 42,ST77XX_BLACK);                 // Blank out old frequency
    tft.print(tempfreq,1); 
    tft.setTextSize(1);
    tft.setCursor(170, 140);                                  // Set cursor
    tft.print("MHz FM"); 
    resetFont();  
      break;
    case AM_BAND:
    
      break;
    case LW_BAND:
     
      break;
    case SW_BAND:
   break;
  }
  //matrix.writeDisplay();                                  // Update LED display
}
/***************************************************************************************************
*                  displayMenu(): display selected menu             *
***************************************************************************************************/
void displayMenu(int8_t menuNumber)
{
  tft.setTextColor(ST77XX_BLACK);             // Use gren for RDS message
  tft.setFont(&FreeSans9pt7b);           // Set to nice font
  tft.setTextSize(1);                         // Native size
  switch (menuNumber)
    {  
      case 0:                                             // Band menu
        for (int8_t i=0;i<=5;i++)
        {
          tft.setCursor(2,45+(i*20));
          if (i==currentBandIndex)
          {
            tft.fillRect(2,30+(i*20),128,18,0x0518); // Clear RDS indicator
            tft.setTextColor(ST77XX_WHITE);             // Use gren for RDS message 
          }
          else
          {
            tft.fillRect(2,30+(i*20),128,18,LTGRAY); // Clear RDS indicator
            tft.setTextColor(ST77XX_BLACK);             // Use gren for RDS message 
          }
          tft.print(band[i].bandName);
        } 
      break;
      case 1:
      break;
      case 5:  
        showFirmwareInformation(); 
      break;
    }
 resetFont();
}

/***************************************************************************************************
*   displayPopUp(uint8_t headerNumber): display PopUp Menu with appropriate header (headNumber)    *
***************************************************************************************************/
/*********************************************************|    ************************************/
void displayPopUp(uint8_t headerNumber)
{
  tft.fillRect(0,1,172,150,LTGRAY);                       // Draw the menu box
  tft.fillRect(0,1,172,24,SKYBLUE);                       // Draw the header box
  tft.setTextColor(ST77XX_BLACK);                         // Use black for header text
  tft.setFont(&FreeSans9pt7b);                            // Set to nice font
  tft.setTextSize(1);                                     // Native size
  tft.setCursor(2+mHeader[menuNumber].menuHeadOffset,18); // Positon cursor
  tft.print(mHeader[headerNumber].menuHeader);            // Display the appropriate header
  resetFont();
}
/***************************************************************************************************
*                  displayReticle(): display current frequency range reticle                       *
***************************************************************************************************/
void displayReticle()
{
  uint16_t lineColor=0xC638;                              // Reticle line color
  tft.setTextWrap(false);                                 // Clip frequency text
  tft.setTextColor(ST77XX_WHITE);                         // Use white for frequencies
  tft.fillRect(0,55,319,30, ST77XX_BLACK);                // Clear the reticle area
  int temp=(currentFrequency/10.00)-20;              
  for(int i=0;i<40;i++)                                   // Loop to draw reticle
  {
    if (i==20) lineColor=ST77XX_RED;                      // If center of screen use red
    else lineColor=0xC638;                                // Otherwise use default
    if (!(temp<band[bandIdx].minimumFreq/10.00 or temp>band[bandIdx].maximumFreq/10.00)) 
    {
      if((temp%10)==0)
      {
        lineColor=ST77XX_WHITE;
        tft.drawLine(i*8,90,i*8,70,lineColor);
        tft.drawLine((i*8)+1,90,(i*8)+1,70,lineColor);
        tft.setCursor(i*8-27, 55);
        if (currentBandMode == FM_BAND)                   // Format frequcy for FM
        {
          tft.setCursor(i*8-27, 55);
          tft.print(temp/10.0,1);
        }
        else if (currentBandMode == AM_BAND)              // Format frequcy for AM
        {
          tft.setCursor(i*8-22, 55);
          tft.print(temp*10); 
        }   
        else if (temp >= 100)                             // Format frequcy for SW
        {
          tft.setCursor(i*8-27, 55);
          tft.print(temp/100.0,3);
        }
        else 
        {
          tft.setCursor(i*8-27, 55);
          tft.print(temp*10,2);
        }
      } else if((temp%5)==0 && (temp%10)!=0) {
        tft.drawLine(i*8,90,i*8,70,lineColor);
        tft.drawLine((i*8)+1,90,(i*8)+1,70,lineColor);        
      } 
      else if (i==20){
        tft.drawLine(i*8,90,i*8,65,lineColor);}
      else {
        tft.drawLine(i*8,90,i*8,80,lineColor);
      }
    }
   temp=temp+1;
  }
}
/***************************************************************************************************
*                  displayRDS(): display a new RDS message                                         *
***************************************************************************************************/
void displayRDS()
{
  tft.fillRect(0,25,158,25,ST77XX_BLACK);                 // Clear RDS indicator
  tft.setTextColor(ST77XX_GREEN);                         // Use gren for RDS message
  tft.setFont(&FreeSans12pt7b);                           // Set to nice font
  tft.setTextSize(1);                                     // Native size
  tft.setCursor(2,45);                                    // Positon cursor
  tft.print(rdsMessage);                                  // Display the message
  resetFont();
}
/***************************************************************************************************
*                  displayRadioScreen(): display the defaul Start(ing) user Screen                 *
***************************************************************************************************/        
void displayRadioScreen()
{
  tft.drawFastVLine(159, 1, 50, ST77XX_WHITE);            // Band + Mode and RDS/Info Seperator
  tft.drawFastHLine(0,50,319,ST77XX_WHITE);               // Top seperator line
  tft.drawRect (282, 1, 33, 12, ST77XX_WHITE);            // Battery Outline
  tft.fillRect (315,3,3,8,ST77XX_WHITE);                  // Battery "cap"
  displayButtonBar(1);                                    // Label Button Bar
  displayFrequency(currentFrequency);                     // Display frequency on LED's
  displayReticle();                                       // Draw frequency reticle
  displayBandMode();                                      // Display current band and recieve mode
  tft.setFont(&FreeSans9pt7b);                            // Set to nice font
  tft.setTextSize(1);                                     // Base text size
  tft.setTextColor(LTGRAY);                               // Use light gray for text
  tft.setCursor(167, 14);                                 // Set postion for volume label
  tft.print("VOL:");                                      // Draw volume label 
  tft.setCursor(166, 31);                                 // Set postion for volume label
  tft.print("SNR:");                                      // Draw volume label 
  resetFont();                                            // Reset to system font
  displayVolume();                                        // Display current volume level

}
/***************************************************************************************************
*                  displayRSSI(): Display Recieved Signal Strength (Indicator)                     *
***************************************************************************************************/
void displayRSSI()
{
  uint16_t tempColor;                                     // Temporary varialble for bar color
  rx.getCurrentReceivedSignalQuality();                   // Fetch receiver data
  rssi = rx.getCurrentRSSI();                             // Get current RSS
  if (rssi==previousRSSI) return;                          // Check to see if update is required
  tft.fillRect(255,19,64,20,ST77XX_BLACK);                // Clear RSS Indicator
  previousRSSI=rssi;                                      // Update the RSS value
  for (int i = 0; i < getStrength(rssi); i++)             // Based on log conversion display RSSI
  {
    tempColor=ST77XX_RED;                                 // If weak display bar in red
    if (i>=4) tempColor=ST77XX_YELLOW;                    // If medium display bar in yellow
    if (i >= 8) tempColor=ST77XX_GREEN;                   // If strong display bar in green                                      
    tft.fillRect(255+(i*4), 35-i, 2, 4+i, tempColor);     // Display the bar
  } 
} 
/*******************************************************************************
*        displaySNR(): Display Signal to Noise Ratio (0-128dB)      *
*******************************************************************************/
void displaySNR()
{
  rx.getCurrentReceivedSignalQuality();                   // Fetch receiver data 
  uint8_t snr=rx.getCurrentSNR();                         // Get current Signal Noise Ratio
  if (snr==previousSNR) return;
  tft.setFont(&FreeSans9pt7b);                            // Set to nice font
  tft.setTextSize(1);                                     // Base text size
  tft.setTextColor(LTGRAY);                               // Use white for text
  tft.fillRect(210,18,60,14,ST77XX_BLACK);                // Wipe out previous SNR
  tft.setCursor(210,31);                                 // Set postion for SNR
  tft.print(snr);                                         // Display SNR
  tft.print("dB");                                        // Display unit
  resetFont();
} 
/***************************************************************************************************
*                  displayStartup(): Display radio startup screen & HELLO on LED's                 *
***************************************************************************************************/
void displayStartup()
{  
  
  tft.fillScreen(ST77XX_BLACK);                           // Clear TFT display
  tft.setCursor(1, 1);                                    // Initialize cursor
  tft.setTextSize(2);                                     // Set text to 10x14
  tft.setTextColor(ST77XX_GREEN);                         // Green text
  tft.print("Si4735 Feather   V");                        // Beginning of SW Version  
  tft.println(Version);                                   // SW Version
  tft.drawFastHLine(1,18,300,ST77XX_WHITE);               // Seperator line
  displayButtonBar(0);                                    // Label Button Bar
  while(digitalRead(L_BUTTON)){}                    // Wait for button press
  tft.fillScreen(ST77XX_BLACK);                           // Clear screen and continue
}      
/*******************************************************************************
*            displayStatus(): update the status box on the TFT                 *
*******************************************************************************/ 
/*********************************************************|    ************************************/   
void displayStatus()
{
  //tft.setTextSize(2);                       // All text in a 10x14 font
  //tft.setTextColor(ST77XX_WHITE);           // Use white for text
  //tft.fillRect(65,5,40,16, ST77XX_BLACK);
  //tft.setCursor(65,5);                      // Current band VHF, 20m etc
  //tft.print(band[bandIdx].bandName);
  //tft.fillRect(65,25,40,16, ST77XX_BLACK);
  //tft.setCursor(65,25); 
  //tft.print(modemenu[band[bandIdx].bandType].itemName);// Current mode - FM,AM etc
  //tft.setCursor(65,65);
  //tft.print(band[bandIdx].currentStepIdx);// Current tune increment 1k,10k etc
  //tft.setCursor(65,85);
 /// tft.print(bwcfSettings[currentBWCH].itemName);// Current bandwidth filter
  //tft.setCursor(65,105);
  //tft.print("ON");                            // Current AGC mode
  //tft.setCursor(65,105);
  //tft.print("OFF");                           // Current BFO mode
  //displayBandMode();
  }

/*******************************************************************************
*        displayVolume(): Display Si4735 volume as a percentage (of 0-64)      *
*******************************************************************************/
void displayVolume()
{
  float currentVolume;                                    // Temporary variable for volume

  currentVolume=rx.getCurrentVolume();                    // Get current volume level

  currentVolume=currentVolume/63*100;                     // Calculate percent of full volume
  tft.setFont(&FreeSans9pt7b);                            // Set to nice font
  tft.setTextSize(1);                                     // Base text size
  tft.setTextColor(LTGRAY);                               // Use white for text
  tft.fillRect(210,1,60,14,ST77XX_BLACK);                 // Wipe out previous volume
  tft.setCursor(210, 14);                                 // Set postion for volume
  tft.print(currentVolume,0);                             // Display volume 
  tft.print("%");                                         // Display unit
  resetFont();                                            // Reset to system font
} 
/***************************************************************************************************
*     getStrength(): convert recieved signal strength to log VU equivalent and return value        *
***************************************************************************************************/
int getStrength(uint8_t rssi)
{
  for(int i=0;i<=15;i++)
  {
    if((rssi>=logLimits[i]) & (rssi<=logLimits[i+1])) return i+1;  
  }
  return 0;
}
/***************************************************************************************************
*     haltCatchFire(): If radio chip is not found, display error message & halt                    *
***************************************************************************************************/
void haltCatchFire()
{
  tft.fillScreen(ST77XX_BLACK);                           // Clear screen and continue
  tft.setFont(&FreeSansBold12pt7b);                       // Set to nice font
  tft.setTextSize(2);                                     // Base text size
  tft.setCursor(90, 35);                                  // Set cursor
  tft.setTextColor(ST77XX_WHITE);                         // In White
  tft.println("Si4732");                                  // Halt & catch fire message
  tft.setCursor(35, 90);                                  // Set cursor
  tft.println("Not Found");                               // Halt & catch fire message
  tft.setTextColor(ST77XX_RED);                           // In red
  tft.setCursor(70, 150);                                 // Set cursor
  tft.print("HALTED");                                    // Halt & catch fire message
  while (1);                                              // Wait forever
}
/***************************************************************************************************
*                  i2cScan(): Scan each of 127 i2c addresses across for devices                    *
***************************************************************************************************/
/*********************************************************|    ************************************/
void i2cScan()
{
  byte error, Device_addr;                    //variable for error and I2C address
  int nDevices=0; 
  boolean i2CDeviceIDFlag=false;
  for (Device_addr = 0; Device_addr <= 127; Device_addr++ )
    {
    Wire1.beginTransmission(Device_addr);
    error = Wire1.endTransmission();
    if (error == 0)
      {
        tft.setCursor(1, 38+(18*nDevices));
        tft.print("0x"); 
        if (Device_addr < 16) tft.print("0");
        tft.print(Device_addr, HEX);
        tft.print(":");
           
      for (int j=0;j<Numi2CDev;j++)
      {
        if (i2cDev[j].i2c_chan==Device_addr) 
        {
            tft.println(i2cDev[j].I2CDev);
            i2CDeviceIDFlag=true;
          
        nDevices++;  
        } 
      }
      if (!i2CDeviceIDFlag) 
      {
        tft.println(F("Unknown"));
        nDevices++;
      }
     i2CDeviceIDFlag=false;
    } 
  } 

}
/***************************************************************************************************
*         loadSSB(): (up)load SSB patch into the Si4735 or Si4732 recievers                        *
***************************************************************************************************/
void loadSSB()
{

  rx.setI2CFastModeCustom(500000);                    // Fast i2C speed mode
  rx.queryLibraryId();                                // Is it really necessary here? I will check it.
  rx.patchPowerUp();
  delay(50);
  rx.downloadCompressedPatch(ssb_patch_content, size_content, cmd_0x15, cmd_0x15_size);
  rx.setSSBConfig(bandwidthSSB[bwIdxSSB].idx, 1, 0, 1, 0, 1);
  rx.setI2CStandardMode();                            // Return i2C to standard speed

  ssbLoaded = true;
}
/***************************************************************************************************
*   resetFont(): reset from graphics font to system font                                           *
***************************************************************************************************/
void resetFont()
  {
  tft.setFont(NULL);                                      // Reset font to system font
  tft.setTextSize(2);                                     // Reset system font size
  tft.setTextColor(ST77XX_WHITE);                         // Reset system text color
  }
/***************************************************************************************************
*   rotaryEncoder(): interrupt handler for Tune rotary encoder - MUST BE AS SMALL AS POSSIBLE     *
***************************************************************************************************/
void rotaryEncoder()
{ // rotary encoder events
  uint8_t encoderStatus = encoder.process();         // Get the status of the tune encoder
  if (encoderStatus)
    encoderCount = (encoderStatus == DIR_CW) ? 1 : -1; // if Clockwise set to 1 else -1
}

/***************************************************************************************************
*   showFirmwareInformation(): display technical information on Si4735 radio                       *
***************************************************************************************************/
void showFirmwareInformation() 
{

  rx.getFirmware();
  tft.setCursor(1, 38);
  tft.print("Part #: 0x");
  tft.println(rx.getFirmwarePN(), HEX);
  tft.print("Firmware: ");
  tft.print(rx.getFirmwareFWMAJOR());
  tft.print(".");
  tft.println(rx.getFirmwareFWMINOR());
  tft.print("Patch ID: ");
  tft.print(rx.getFirmwarePATCHH(), HEX);
  tft.println(rx.getFirmwarePATCHL(), HEX);
  tft.print("Component: ");
  tft.print(rx.getFirmwareCMPMAJOR());
  tft.print(".");
  tft.println(rx.getFirmwareCMPMINOR());
  tft.print("Chip Revision: ");
  tft.println(rx.getFirmwareCHIPREV());

}
