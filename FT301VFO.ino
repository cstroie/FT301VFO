/*
   ------------------------------------------------------------------------
   "PH2LB LICENSE" (Revision 1) : (based on "THE BEER-WARE LICENSE" Rev 42)
   <lex@ph2lb.nl> wrote this file. As long as you retain this notice
   you can do modify it as you please. It's Free for non commercial usage
   and education and if we meet some day, and you think this stuff is
   worth it, you can buy me a beer in return
   Lex Bolkesteijn
   ------------------------------------------------------------------------
   Filename : FT301VFO.ino
   Version  : 0.5 (DRAFT)
   ------------------------------------------------------------------------
   Description : A Arduino DDS based VFO for the Yaesu FT301.
   ------------------------------------------------------------------------
   Revision :
    - 2016-jan-26 0.1 initial version
    - 2016-jan-28 0.2 playing around and added encoder (on A2 and A3)
    - 2016-okt-15 0.3 added support for AD9833
    - 2016-okt-19 0.4 added support for MAX7219

    - 2017-may-8  0.5 added support for SI5351A and updated the rotary encoder support
   ------------------------------------------------------------------------
   Hardware used :
    - Arduino Uno R3
    - QRP-Labs Arduino Shields (http://qrp-labs.com/uarduino.html)
    - LCD keypad shield (http://www.hobbytronics.co.uk/arduino-lcd-keypad-shield)
      (warning : don't use D10 with this shield)
      or
    - MAX7219 with analoge button
    - Simpel rotary encoder
   ------------------------------------------------------------------------
   Software used :
    - AD9850 library from Poul-Henning Kamp
    - LedControl library from Eberhard Fahle
   ------------------------------------------------------------------------
   TODO LIST :
    - add more sourcode comment
    - add debounce for the keys
    - harden the rotary switch (sometimes strange behaviour due to timing
      isues (current no interrupt).
   ------------------------------------------------------------------------
*/


// use just one of the DDS models (AD9850 or AD9833)
//#define USE_AD9850 1
//#define USE_AD9833 1
#define USE_SI5351A 1
// use one or both display options
//#define USE_LEDCONTROL
#define USE_LCD 1

#ifdef USE_LCD
#include <LiquidCrystal.h>
#endif
#ifdef USE_AD9850
#include "AD9850.h"
#endif
#ifdef USE_AD9833
#include "AD9833.h"
#endif
#ifdef USE_SI5351A
#include "SI5351A.h"
#endif
#ifdef USE_LEDCONTROL
#include "LedControl.h"
#endif

#ifdef USE_AD9850
AD9850 ad(3, A5, A4); // w_clk, fq_ud, d7
#endif

#ifdef USE_AD9833
AD9833 ad9833 = AD9833(11, 13, A1); // SPI_MOSI, SPI_CLK, SPI_CS
#endif

#ifdef USE_SI5351A
SI5351A si5351a = SI5351A();
#endif

#ifdef USE_LEDCONTROL
LedControl lc = LedControl(11, 13, 2, 1); // SPI_MOSI, SPI_CLK, SPI_CS, NR OF DEVICES  NOTE TO MY SELF>> CS is now A1 instead of 12 (12=MISO on UNO)
#endif

// Display shield :
#ifdef USE_LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Check your pinount
#endif
#define buttonAnalogInput A0   // A0 on the LCD shield (save some pins)

// FT-301 VFO limits (5Mhz - 5.5Mhz)
uint32_t vfofreqlowerlimit = 5e6;
uint32_t vfofrequpperlimit = 55e5;

// for a little debounce (needs to be better than now)
int_fast32_t prevtimepassed = millis();
int buttonState;            // the current reading from the input pin
int lastButtonState = 0;    // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 4;     // the debounce time; increase if the output flickers

// define the bandstruct
typedef struct {
  char * Text;
  uint32_t FreqLowerLimit; // lower limit acording to band plan
  uint32_t FreqUpperLimit; // upper limit according to bandplan
  uint32_t FreqBase;       // the base frequency (first full 500Khz block from FreqLowerLimit)
  uint32_t Freq;           // the current frequency on that band (set with default)
} BandStruct;

// define the band enum
typedef enum {
  BANDMIN = 0,
  B160M = 0,
  B80M = 1,
  B40M = 2,
  B30M = 3,
  B20M = 4,
  B17M = 5,
  B15M = 6,
  B12M = 7,
  B10M = 8,
  BANDMAX = 8
} BandEnum;

// define the bandstruct array (PA country full-license HF Band plan)
BandStruct Bands[] = {
  {(char *)"160m", (uint32_t)1810e3,  (uint32_t)1880e3,  (uint32_t)1500e3,  (uint32_t)1840e3}, // (uint32_t) to prevent > warning: narrowing conversion of '1.86e+6' from 'double' to 'uint32_t {aka long unsigned int}' inside { } [-Wnarrowing]
  {(char *)" 80m", (uint32_t)3500e3,  (uint32_t)3800e3,  (uint32_t)3500e3,  (uint32_t)3573e3}, // (char *) to prevent > warning: deprecated conversion from string constant to 'char*' [-Wwrite-strings]
  {(char *)" 40m", (uint32_t)7000e3,  (uint32_t)7300e3,  (uint32_t)7000e3,  (uint32_t)7074e3},
  {(char *)" 30m", (uint32_t)10100e3, (uint32_t)10150e3, (uint32_t)10100e3, (uint32_t)10136e3},
  {(char *)" 20m", (uint32_t)14000e3, (uint32_t)14350e3, (uint32_t)14000e3, (uint32_t)14074e3},
  {(char *)" 17m", (uint32_t)18068e3, (uint32_t)18168e3, (uint32_t)18068e3, (uint32_t)18100e3},
  {(char *)" 15m", (uint32_t)21000e3, (uint32_t)21450e3, (uint32_t)21000e3, (uint32_t)21074e3},
  {(char *)" 12m", (uint32_t)24890e3, (uint32_t)24990e3, (uint32_t)24890e3, (uint32_t)24915e3},
  {(char *)" 10m", (uint32_t)28000e3, (uint32_t)29700e3, (uint32_t)28000e3, (uint32_t)28074e3},
};

// define the stepstruct
typedef struct {
  char * Text;
  uint32_t Step;
} StepStruct;

// define the step enum
typedef enum {
  STEPMIN = 0,
  S10 = 0,
  S25 = 1,
  S100 = 2,
  S250 = 3,
  S1KHZ = 4,
  S2_5KHZ = 5,
  S10KHZ = 6,
  STEPMAX = 6
} StepEnum;

// define the bandstruct array
StepStruct Steps[] = {
  {(char *)"  10Hz", 10},
  {(char *)"  25Hz", 25},
  {(char *)" 100Hz", 100},
  {(char *)" 250Hz", 250},
  {(char *)"  1kHz", 1000},
  {(char *)"2.5kHz", 2500},
  {(char *)" 10kHz", 10000}
};

// Switching band stuff
boolean switchBand = false;
int currentBandIndex = (int)B40M;       // my default band
int currentFreqStepIndex = (int)S1KHZ;  // default step 1kHz


// Encoder stuff
int encoder0PinALast = LOW;
#define encoderPin1 2
#define encoderPin2 3
volatile uint8_t lastEncoded = 0;
volatile int16_t encoderValue = 0;
int16_t lastencoderValue = 0;
uint8_t lastMSB = 0;
uint8_t lastLSB = 0;
boolean ccw = false;
boolean cw  = false;


#define debugSerial Serial
#define debugPrintLn(...) {if (debugSerial) debugSerial.println(__VA_ARGS__);}
#define debugPrint(...) {if (debugSerial) debugSerial.print(__VA_ARGS__);}


#define STEPUP     50
#define STEPDOWN  450
#define BANDUP    150
#define BANDDOWN  300
#define SELECT    700

// RT 0
// UP 101
// DN 257
// LT 410
// SL 640

int nrOfSteps = 0;

// LCD stuff
boolean updatedisplayfreq = false;
boolean updatedisplaystep = false;
boolean updatedisplaybandselect = false;

// Playtime
void setup() {
  debugSerial.begin(115200);
  debugPrintLn(F("FT301 DDS VFO"));

#if defined(USE_AD9833) || defined(USE_LEDCONTROL)
  SPI.begin();
  SPI.setDataMode(SPI_MODE2);
#endif

#ifdef USE_LEDCONTROL
  lc.init();
  /*
    The MAX72XX is in power-saving mode on startup,
    we have to do a wakeup call
  */
  lc.shutdown(0, false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0, 6);
  /* and clear the display */
  lc.clearDisplay(0);

  writeTextToLed("FT301DDS");
  delay(1000);
#endif

#ifdef USE_LCD
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print(F(" FT301  DDS VFO "));
  lcd.setCursor(0, 1);
  lcd.print(F(" PH2LB & YO9JAZ "));
  delay(1000);
#endif

#ifdef USE_AD9833
  ad9833.init();
  ad9833.reset();
#endif

  updatedisplayfreq = true;
  updatedisplaystep = true;

  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); // turn pullup resistor on

  // call updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

#ifdef USE_LCD
  lcd.setCursor(0, 0);
  lcd.print(F(" FT301  DDS VFO "));
  lcd.setCursor(0, 1);
  lcd.print(F("     Ready      "));
  delay(1000);
#endif

#ifdef USE_LEDCONTROL
  writeTextToLed("ready");
  delay(1000);
#endif

  // Update the displays
  updateDisplays();
}


void updateEncoder() {
  uint8_t MSB = digitalRead(encoderPin1); // MSB = most significant bit
  uint8_t LSB = digitalRead(encoderPin2); // LSB = least significant bit

  uint8_t encoded = (MSB << 1) | LSB;           // converting the 2 pin value to single number
  uint8_t sum  = (lastEncoded << 2) | encoded;  // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  // Store this value for next time
  lastEncoded = encoded;
}

// Set the AD9850 to the VFO frequency depending on bandplan.
void setFreq() {
#ifdef USE_AD9850
  ad.setfreq(Bands[currentBandIndex].Freq);
#endif

#ifdef USE_AD9833
  ad9833.setFrequency((long)Bands[currentBandIndex].Freq);
#endif

#ifdef USE_SI5351A
  si5351a.CLK0SetFrequency((long)Bands[currentBandIndex].Freq);
#endif
}

#ifdef USE_LEDCONTROL
void writeTextToLed(char *p) {
  lc.clearDisplay(0);
  for (uint8_t i = 0; i < 8; i++)
    lc.setChar(0, i, p[7 - i], false);
}

void freqToLed(long v) {
  lc.clearDisplay(0);
  // Variable value digit
  int digit[8] = 0;
  // Calculate the value of each digit
  for (uint8_t i = 0; i < 8; i++) {
    digit[i] = v % 10;
    v /= 10;
  }
  // Display the value of each digit in the display
  for (uint8_t i = (digit[7] > 0 ? 7 : 6); i >= 0; i--) {
    lc.setDigit(0, i, (byte)digit[i], (i == 6 or i == 3) ? true : false);
  }
}

void bandToLed(char *p) {
  // it's save because Text = atleast 4 chars (check the declare)
  lc.clearDisplay(0);

  lc.setChar(0, 7, 'B', false);
  lc.setChar(0, 6, 'a', false);
  lc.setChar(0, 5, 'n', false);
  lc.setChar(0, 4, 'd', true);

  if (p == NULL || strlen(p) < 4) {
    lc.setChar(0, 3, 'E', false);
    lc.setChar(0, 2, 'R', false);
    lc.setChar(0, 1, 'R', false);
  }
  else {
    lc.setChar(0, 3, p[0], false);
    lc.setChar(0, 2, p[1], false);
    lc.setChar(0, 1, p[2], false);
    lc.setChar(0, 0, p[3], false);
  }
}

void stepToLed(long s) {
  lc.clearDisplay(0);
  lc.setChar(0, 7, 's', false);
  lc.setChar(0, 6, 't', false);
  lc.setChar(0, 5, 'e', false);
  lc.setChar(0, 4, 'p', true);

  // Variable value digit
  int digito1;
  int digito2;
  int digito3;
  int digito4;
  boolean is10Kplus = false;

  if (s >= 1000) {
    s = s / 100;
    is10Kplus = true;
  }

  // Calculate the value of each digit
  digito1 = s %  10 ;
  digito2 = (s / 10) % 10 ;
  digito3 = (s / 100) % 10 ;
  digito4 = (s / 1000) % 10 ;

  // Display the value of each digit in the display
  if (s >= 1000)
    lc.setDigit(0, 3, (byte) digito4, false);
  if (s >= 100)
    lc.setDigit(0, 2, (byte) digito3, false);
  // there isn't anything smaller then 10
  lc.setDigit(0, 1, (byte) digito2, is10Kplus);
  lc.setDigit(0, 0, (byte) digito1, false);
}
#endif // USE_LEDCONTROL

// Update the LCD
void updateDisplays() {
  if (updatedisplayfreq) {
    // because we display kHz and the Freq is in MHz.
    float freq = Bands[currentBandIndex].Freq / 1e3;

#ifdef USE_LCD
    lcd.setCursor(0, 0);
    lcd.print(F("  "));
    if (freq < 10e3)
      lcd.print(F(" "));
    lcd.print(freq);
    lcd.print(F(" kHz  "));
#endif

#ifdef USE_LEDCONTROL
    freqToLed(Bands[currentBandIndex].Freq);
#endif
  }

  if (updatedisplaystep) {
#ifdef USE_LCD
    lcd.setCursor(0, 1);
    lcd.print(F("  "));
    lcd.print(Bands[currentBandIndex].Text);
    lcd.print(F("  "));
    lcd.print(Steps[currentFreqStepIndex].Text);
    lcd.print(F("  "));
#endif

#ifdef USE_LEDCONTROL
    stepToLed(Steps[currentFreqStepIndex].Step);
#endif
  }

  if (updatedisplaybandselect) {
    updatedisplaybandselect = false;
#ifdef USE_LCD
    lcd.setCursor(0, 0);
    lcd.print(F("  Band select   "));
    lcd.setCursor(0, 1);
    lcd.print(F("                "));
    lcd.setCursor(0, 1);

    if (currentBandIndex > BANDMIN)
      lcd.print(Bands[currentBandIndex - 1].Text);
    lcd.print(F(" "));
    lcd.print(F("["));
    lcd.print(Bands[currentBandIndex].Text);
    lcd.print(F("]"));
    lcd.print(F(" "));
    if (currentBandIndex < BANDMAX)
      lcd.print(Bands[currentBandIndex + 1].Text);
#endif

#ifdef USE_LEDCONTROL
    bandToLed(Bands[currentBandIndex].Text);
#endif
  }
}


// the main loop
void loop() {
  boolean updatefreq = false;
  int reading;

  updatedisplayfreq = false;
  updatedisplaystep = false;

  nrOfSteps = encoderValue;

  encoderValue = 0;

  // int to hold the arduino miilis since startup
  int_fast32_t timepassed = millis();
  if (((timepassed - prevtimepassed) > 10 && !switchBand) ||
      ((timepassed - prevtimepassed) > 25 && switchBand) ||
      (prevtimepassed > timepassed) ||
      nrOfSteps != 0) {

    ccw = false;
    cw = false;
    if (nrOfSteps != 0) {
      debugPrintLn(nrOfSteps);
      if (nrOfSteps < 0)
        ccw = true;
      else
        cw = true;
    }

    prevtimepassed = timepassed;
    // Check if a button is pressed
    reading = analogRead(buttonAnalogInput);
    //debugPrintLn(reading);

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;

        if (reading < STEPUP) {
          // Right
          debugPrintLn("STEPUP");
          if (switchBand) {
            if (currentBandIndex < BANDMAX) {
              currentBandIndex = (BandEnum)currentBandIndex + 1;
              updatedisplaybandselect = true;
            }
          }
          else {
            if (currentFreqStepIndex < STEPMAX) {
              currentFreqStepIndex = (StepEnum)currentFreqStepIndex + 1;
              updatedisplaystep = true;
            }
          }
        }
        else if (reading < BANDUP || cw) {
          // Up
          debugPrintLn("BANDUP");
          if (switchBand) {
            if (currentBandIndex < BANDMAX) {
              currentBandIndex = (BandEnum)currentBandIndex + 1;
              updatedisplaybandselect = true;
            }
          }
          else {
            if (Bands[currentBandIndex].Freq < Bands[currentBandIndex].FreqUpperLimit) {
              Bands[currentBandIndex].Freq = Bands[currentBandIndex].Freq + Steps[currentFreqStepIndex].Step;
              updatefreq = true;
              updatedisplayfreq = true;
            }
            if (Bands[currentBandIndex].Freq > Bands[currentBandIndex].FreqUpperLimit) {
              Bands[currentBandIndex].Freq = Bands[currentBandIndex].FreqUpperLimit;
            }
          }
        }
        else if (reading < BANDDOWN || ccw) {
          // Down
          debugPrintLn("BANDDOWN");
          if (switchBand) {
            if (currentBandIndex > BANDMIN) {
              currentBandIndex = (BandEnum)currentBandIndex - 1;
              updatedisplaybandselect = true;
            }
          }
          else {
            if (Bands[currentBandIndex].Freq > Bands[currentBandIndex].FreqLowerLimit) {
              Bands[currentBandIndex].Freq = Bands[currentBandIndex].Freq - Steps[currentFreqStepIndex].Step;
              updatefreq = true;
              updatedisplayfreq = true;
            }
            if (Bands[currentBandIndex].Freq < Bands[currentBandIndex].FreqLowerLimit) {
              Bands[currentBandIndex].Freq = Bands[currentBandIndex].FreqLowerLimit;
            }
          }
        }
        else if (reading < STEPDOWN) {
          // Left
          debugPrintLn("STEPDOWN");
          if (switchBand) {
            if (currentBandIndex > BANDMIN) {
              currentBandIndex = (BandEnum)currentBandIndex - 1;
              updatedisplaybandselect = true;
            }
          }
          else {
            if (currentFreqStepIndex > STEPMIN) {
              currentFreqStepIndex = (StepEnum)currentFreqStepIndex - 1;
              updatedisplaystep = true;
            }
          }
        }
        else if (reading < SELECT) {
          // Select
          debugPrintLn("SELECT");
          if (switchBand) {
            switchBand = false;
            // Update everything
            updatedisplayfreq = true;
            updatedisplaystep = true;
            updatefreq = true;
          }
          else {
            switchBand = true;
            updatedisplaybandselect = true;
          }
        }
      }
    }
  }

  // Save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
  // Update the frequency, if needed
  if (updatefreq)
    setFreq();
  // Update the display
  updateDisplays();
}
