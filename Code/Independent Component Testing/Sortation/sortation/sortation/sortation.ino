#define PRINT_COLOUR                                                   // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();

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

int startPos = 850;

// Define servo movement steps
int ServoStepsToLeft[] = {
  startPos, 880, 860, 840, 820, 800, 780, 760, 740, 720,
  700, 680, 660, 640, 620, 600, 580, 560, 400, 400,
  400, 400, 400, 400, 400, 400,
  400, 400, 400, 400, 400, 400, 400, 400, 560, 580,
  600, 620, 640, 660, 680, 700, 720, 740, 760, 780,
  800, 820, 840, 860, 880, startPos
};

int ServoStepsToRight[] = {
  startPos, 919, 938, 957, 976, 995, 1014, 1033, 1052, 1071,
  1090, 1109, 1128, 1147, 1166, 1185, 1204, 1223, 1242, 1261,
  1280, 1299, 1318, 1337, 1356, 1500,
  1500, 1356, 1337, 1318, 1299, 1280, 1261, 1242, 1223, 1204,
  1185, 1166, 1147, 1128, 1109, 1090, 1071, 1052, 1033, 1014,
  995, 976, 957, 957, 957, startPos
};


const int numSteps = sizeof(ServoStepsToLeft) / sizeof(ServoStepsToLeft[0]);

// to reset board every once in a while
// to prevent current surge/draw issues that stall the servo
int sortCycles = 0;


const int cPotPin            = 36;                                     // GPIO pin for drive speed potentiometer (A0)


void setup() {
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor

  pinMode(cPotPin, INPUT);                                             // set up drive speed potentiometer


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
  ledcWrite(servoPin, startPos);

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
    int pot = 0; // read raw analog value from potentiometer

    static unsigned long lastStepTime = 0; // keeps track of time for steps
    static int step = 0; // current step in the state machine
    static bool movingLeft = false; // tracks movement direction
    unsigned long currentMillis = millis(); // get current time

    pot = analogRead(cPotPin); // read potentiometer value
    startPos = map(pot, 0, 4095, 700, 1000); // map pot value to servo range

    switch (step) {
      case 0: // color detection step
          if (currentMillis - lastStepTime >= colorCheckInterval) {
              lastStepTime = currentMillis;

              uint16_t r, g, b, c;
              digitalWrite(cTCSLED, !digitalRead(cLEDSwitch)); // toggle led

              if (tcsFlag) {
                  tcs.getRawData(&r, &g, &b, &c); // get color values
                  g = g - 4;
                  b = b - 2;

                  // print raw color values if enabled
                  #ifdef PRINT_COLOUR
                  Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
                  #endif

                  // check if color reading is bright enough
                  if (c > 30) {
                      float sum = r + g + b;
                      float rRatio = r / sum;
                      float gRatio = g / sum;
                      float bRatio = b / sum;

                      // print color ratios
                      Serial.printf("rRatio: %.2f, gRatio: %.2f, bRatio: %.2f\n", rRatio, gRatio, bRatio);

                      // detect green
                      if (gRatio > rRatio + 0.05 && gRatio > bRatio + 0.05) {
                          Serial.println("Green detected");
                          moving = true;
                          currentPositionIndex = 0;
                          step = 3; // move left
                      } else {
                          Serial.println("Other color detected");
                          moving = true;
                          currentPositionIndex = 0;
                          step = 4; // move right
                      }
                  }
              }
          }
          break;

      case 3: // move servo left step-by-step
          if (moving && currentMillis - lastServoUpdate >= servoUpdateInterval) {
              lastServoUpdate = currentMillis;
              if (currentPositionIndex < numSteps) {
                  ledcWrite(servoPin, ServoStepsToLeft[currentPositionIndex++]);
              } else {
                  moving = false;
                  step = 1;  // go to hold
                  lastStepTime = currentMillis;
              }
          }
          break;

      case 4: // move servo right step-by-step
          if (moving && currentMillis - lastServoUpdate >= servoUpdateInterval) {
              lastServoUpdate = currentMillis;
              if (currentPositionIndex < numSteps) {
                  ledcWrite(servoPin, ServoStepsToRight[currentPositionIndex++]);
              } else {
                  moving = false;
                  step = 1;  // go to hold
                  lastStepTime = currentMillis;
              }
          }
          break;

      case 1: // hold position for a short time
          if (currentMillis - lastStepTime >= 15) {
              lastStepTime = currentMillis;
              Serial.println("Returning to neutral");
              ledcWrite(servoPin, startPos); // go back to start position
              step = 2; // wait before checking color again
          }
          break;

      case 2: // wait 2 seconds before next cycle
          if (currentMillis - lastStepTime >= 2000) {
              lastStepTime = currentMillis;
              step = 0; // go back to color detection

              sortCycles += 1;

              if (sortCycles >= 100) {
                  ESP.restart(); // restart after 100 cycles
              }
          }
          break;
  }

  doHeartbeat(); // blink heartbeat led
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