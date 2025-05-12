/*

Lab 005 - Team 001

Base system code.
This code is responsible for the
color sorting of the gems,
along with the IR beacon.

*/

#define PRINT_COLOUR                                                   // uncomment to turn on output of colour sensor data
#define INCREASE_POWER_USE                                             // Enable if necessary to keep battery on
#define SELF_TEST                                                      // Enable to test with connected IR detector

#ifdef INCREASE_POWER_USE
#include "WiFi.h"
#endif

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();


// Constants for IR
const uint8_t cCarrierPin = 5;                        // GPIO for 38 kHz carrier
const uint8_t cTXPin = 17;                             // GPIO for TX1
const uint8_t cRXPin = 16;                            // GPIO for RX1 (for testing with IR receiver only)
const uint8_t cLEDPin = 2;                            // built-in LED (ON on LOW)
const uint8_t cPWMResolution = 10;                    // bit resolution for carrier PWM (1-14 bits)
const uint8_t cTXData = 0x57;                         // byte to be sent: U = 0b01010101
const uint16_t cNumBytes = 1;                         // number of bytes to transmit in a burst
const uint32_t cBurstInterval = 100;                  // milliseconds between bursts
const uint32_t cPWMFrequency = 38000;                 // carrier frequency in Hz
const float cDutyCycle = 0.5;                         // duty cycle of carrier (0-1)
uint32_t nextBurst;                                   // time of next tranmission burst
bool runState = true;                                 // 0 = stopped; 1 = running
#ifdef INCREASE_POWER_USE
uint32_t nextScan;                                    // time of next scan for WiFi networks
bool scanState = false;                               // 0  = clear network list; 1 = scan for networks
#endif
#ifdef SELF_TEST
uint8_t receivedData = 0;                             // data received from IR receiver (byte)
#endif

#ifdef INCREASE_POWER_USE
const uint16_t cScanInterval = 3000;                  // WiFi network scan interval in milliseconds
#endif
#ifdef SELF_TEST
const uint8_t cIRGndPin = 14;                         // GPIO used for IR receiver ground (testing only)
const uint8_t cIRVCCPin = 13;                         // GPIO used for IR receiver supply (testing only)
#endif

// Constants
const int cHeartbeatInterval = 75;                                     // heartbeat update interval, in milliseconds
const int cSmartLED          = 23;                                     // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                                      // number of Smart LEDs in use
const int cSDA               = 18;                                     // GPIO pin for I2C data
const int cSCL               = 19;                                     // GPIO pin for I2C clock
const int cTCSLED            = 14;                                     // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 34;                                     // DIP switch S1-2 controls LED on TCS32725    
const int servoPin           = 4;                                      // GPIO pin for the servo motor
const int colorCheckInterval = 100;                                    // Time interval (ms) for checking color

// Variables
boolean heartbeatState       = true;                                   // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                                      // time of last heartbeat state change
unsigned long curMillis      = 0;                                      // current time, in milliseconds
unsigned long prevMillis     = 0;                                      // start time for delay cycle, in milliseconds

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
unsigned long lastColorCheck = 0;
bool tcsFlag = 0;                                                      // TCS34725 flag: 1 = connected; 0 = not found

// Smooth servo motion setup
const uint32_t servoUpdateInterval = 50;  // time between position updates (ms)
unsigned long lastServoUpdate = 0;
int targetPosition = 1425;
int currentPositionIndex = 0;
int neutralIndex = 12;  // index corresponding to 1425
bool moving = false;

// Define servo movement steps
int ServoStepsToLeft[] = {
  950, 880, 860, 840, 820, 800, 780, 760, 740, 720,
  700, 680, 660, 640, 620, 600, 580, 560, 400, 400,
  400, 400, 400, 400, 400, 400,
  400, 400, 400, 400, 400, 400, 400, 400, 560, 580,
  600, 620, 640, 660, 680, 700, 720, 740, 760, 780,
  800, 820, 840, 860, 880, 950
};

int ServoStepsToRight[] = {
  950, 919, 938, 957, 976, 995, 1014, 1033, 1052, 1071,
  1090, 1109, 1128, 1147, 1166, 1185, 1204, 1223, 1242, 1261,
  1280, 1299, 1318, 1337, 1356, 1500,
  1500, 1356, 1337, 1318, 1299, 1280, 1261, 1242, 1223, 1204,
  1185, 1166, 1147, 1128, 1109, 1090, 1071, 1052, 1033, 1014,
  995, 976, 957, 938, 919, 950 
};


const int numSteps = sizeof(ServoStepsToLeft) / sizeof(ServoStepsToLeft[0]);


void setup() {
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                                   // initialize smart LEDs object
  SmartLEDs.clear();                                                   // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0));                  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                                          // set brightness [0-255]
  SmartLEDs.show();                                                    // update LED
                 
  Wire.setPins(cSDA, cSCL);                                            // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                                            // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                                   // configure GPIO to set state of TCS34725 LED 

  // Set up servos
  pinMode(servoPin, OUTPUT);                                          // configure arm servo GPIO for output
  ledcAttach(servoPin, 50, 14);                                       // setup arm servo pin for 50 Hz, 14-bit resolution
  ledcWrite(servoPin, 950);

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  pinMode(cLEDPin, OUTPUT);                           // configure built-in LED pin as output
  pinMode(cTXPin, OUTPUT);                            // configure TX pin as output

  #ifdef INCREASE_POWER_USE
    WiFi.mode(WIFI_STA);                                // initialize WiFi in station mode
    WiFi.disconnect();                                  // disconnect from all access points
    delay(100);                                         // ensure WiFi peripheral has started
  #endif
  #ifdef SELF_TEST
    Serial.begin(9600);                                 // enable standard serial to output received bytes
    pinMode(cIRGndPin, OUTPUT);                         // configure IR receiver ground pin as output
    digitalWrite(cIRGndPin, LOW);                       // set low (ground)
    pinMode(cIRVCCPin, OUTPUT);                         // configure IR receiver supply pin as output
    digitalWrite(cIRVCCPin, HIGH);                      // set high (VCC)
  #endif

  // configure LEDC to generate carrier signal with specified frequency, resolution, and duty cycle
  ledcAttach(cCarrierPin, cPWMFrequency, cPWMResolution);
  ledcWrite(cCarrierPin, (1 << cPWMResolution) * cDutyCycle); // 2^cPWMResolution * cDutyCycle

  // configure UART2 with desired parameters: 2400 baud rate, 8 data bits, no parity, 1 stop bit
  Serial1.begin(2400, SERIAL_8N1, cRXPin, cTXPin);
  nextBurst = millis();                               // initialize burst timing
}

void loop() {

  // ir emitting logic

  uint32_t curTime = millis(); // get the current time in milliseconds since the program started

  // check if it's time to send the next ir burst
  if (curTime >= nextBurst) {
    if (runState) { // only send burst if system is active
      for (uint16_t i = 0; i < cNumBytes; i++) {
        Serial1.write(cTXData); // send the data byte to serial1 (connected to ir transmitter)
      }
      digitalWrite(cLEDPin, !digitalRead(cLEDPin)); // toggle the indicator led to show ir burst sent
    }
    nextBurst += cBurstInterval; // schedule the next ir burst
  }

  // optional serial monitor test output
  #ifdef SELF_TEST
    // read and print any incoming data from serial1 (used for testing communication)
    while (Serial1.available() > 0) {
      receivedData = Serial1.read(); // read one byte
      Serial.printf("%c\n", receivedData); // print it to the console
    }
  #endif

  // optional power usage increase by scanning wifi
  #ifdef INCREASE_POWER_USE
    // simulate higher power usage by scanning and deleting wifi networks
    if ((curTime - nextScan) < cScanInterval) {
      nextScan += cScanInterval; // update next scan time
      if (!scanState) {
        WiFi.scanNetworks(true); // start async wifi scan
        scanState = true;
      } else {
        WiFi.scanDelete(); // delete scan results to free memory
        scanState = false;
      }
    }
  #endif


  // color-based servo control logic

  // used to control timing and steps
  static unsigned long lastStepTime = 0;
  static int step = 0; // current step in the sorting process
  static bool movingLeft = false; // keeps track of direction
  unsigned long currentMillis = millis(); // current time again (used for servo timing)

  switch (step) {

    case 0: // detect color
      // check if enough time has passed to perform a new color check
      if (currentMillis - lastStepTime >= colorCheckInterval) {
        lastStepTime = currentMillis;

        uint16_t r, g, b, c; // color values
        digitalWrite(cTCSLED, !digitalRead(cLEDSwitch)); // turn on/off color sensor light

        if (tcsFlag) { // if sensor is ready
          tcs.getRawData(&r, &g, &b, &c); // read raw color data from sensor
          g = g - 3; // adjust green slightly for calibration

          #ifdef PRINT_COLOUR
            Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c); // print raw color values
          #endif

          // only analyze if brightness is high enough
          if (c > 30) {
            float sum = r + g + b;
            float rRatio = r / sum;
            float gRatio = g / sum;
            float bRatio = b / sum;

            Serial.printf("rRatio: %.2f, gRatio: %.2f, bRatio: %.2f\n", rRatio, gRatio, bRatio);

            // decide if the object is green based on ratios
            if (gRatio > rRatio + 0.04 && gRatio > bRatio + 0.04) {
              Serial.println("green detected");
              moving = true; // enable motion
              currentPositionIndex = 0; // reset position index
              step = 3; // move servo left
            } else {
              Serial.println("other color detected");
              moving = true;
              currentPositionIndex = 0;
              step = 4; // move servo right
            }
          }
        }
      }
      break;

    case 3: // move left in smooth steps
      if (moving && currentMillis - lastServoUpdate >= servoUpdateInterval) {
        lastServoUpdate = currentMillis;
        if (currentPositionIndex < numSteps) {
          // gradually step through the positions to left
          ledcWrite(servoPin, ServoStepsToLeft[currentPositionIndex++]);
        } else {
          moving = false; // done moving
          step = 1; // go to hold state
          lastStepTime = currentMillis;
        }
      }
      break;

    case 4: // move right in smooth steps
      if (moving && currentMillis - lastServoUpdate >= servoUpdateInterval) {
        lastServoUpdate = currentMillis;
        if (currentPositionIndex < numSteps) {
          // gradually step through the positions to right
          ledcWrite(servoPin, ServoStepsToRight[currentPositionIndex++]);
        } else {
          moving = false; // done moving
          step = 1; // go to hold state
          lastStepTime = currentMillis;
        }
      }
      break;

    case 1: // hold position for 1 second
      if (currentMillis - lastStepTime >= 1000) {
        lastStepTime = currentMillis;
        Serial.println("returning to neutral");
        ledcWrite(servoPin, 900); // return to center position
        step = 2; // wait before restarting
      }
      break;

    case 2: // wait 2 seconds before checking color again
      if (currentMillis - lastStepTime >= 2000) {
        lastStepTime = currentMillis;
        step = 0; // go back to step 0 and restart color detection
      }
      break;
  }

  doHeartbeat(); // blink status led to show the system is alive
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                                                // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                                         // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                              // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {            // if all defined levels have been used
      LEDBrightnessIndex = 0;                                          // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0));            // set pixel colours to green
    SmartLEDs.show();                                                  // update LED
  }
}