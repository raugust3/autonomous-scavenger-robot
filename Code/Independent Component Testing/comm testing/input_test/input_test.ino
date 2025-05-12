
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>


const int cPowerFlag = 18;                                             // Pin communicating the power state
const int cSweepFlag = 19;                                             // Pin communicating the sweeping direction
const int cBucketFlag = 21;                                            // Pin communicating a bucket cycle

void setup() {
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor

  // Setup inputs from master board
  pinMode(cPowerFlag, INPUT_PULLDOWN); 
  pinMode(cSweepFlag, INPUT_PULLDOWN);
  pinMode(cBucketFlag, INPUT_PULLDOWN); 

}

void loop() {
  Serial.printf("power: %ld, sweep: %ld, bucket: %ld\n", digitalRead(cPowerFlag), digitalRead(cSweepFlag), digitalRead(cBucketFlag));
}
