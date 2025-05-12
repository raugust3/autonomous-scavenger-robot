#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Constants
const uint8_t cCarrierPin = 5;                        // GPIO for 38 kHz carrier
const uint8_t cTXPin = 17;                             // GPIO for TX1
const uint8_t cRXPin = 16;                            // GPIO for RX1 (for testing with IR receiver only)
const uint8_t cLEDPin = 2;                            // built-in LED (ON on LOW)
const uint8_t cPWMResolution = 10;                    // bit resolution for carrier PWM (1-14 bits)
const uint8_t cTXData = 0x57;                         // byte to be sent: W = 0b01010100
const uint16_t cNumBytes = 1;                         // number of bytes to transmit in a burst
const uint32_t cBurstInterval = 100;                  // milliseconds between bursts
const uint32_t cPWMFrequency = 38000;                 // carrier frequency in Hz
const float cDutyCycle = 0.5;                         // duty cycle of carrier (0-1)

// Variables
uint32_t nextBurst;                                   // time of next tranmission burst
bool runState = true;                                 // 0 = stopped; 1 = running

void setup() {
  pinMode(cLEDPin, OUTPUT);                           // configure built-in LED pin as output
  pinMode(cTXPin, OUTPUT);                            // configure TX pin as output

  // configure LEDC to generate carrier signal with specified frequency, resolution, and duty cycle
  ledcAttach(cCarrierPin, cPWMFrequency, cPWMResolution);
  ledcWrite(cCarrierPin, (1 << cPWMResolution) * cDutyCycle); // 2^cPWMResolution * cDutyCycle

  // configure UART2 with desired parameters: 2400 baud rate, 8 data bits, no parity, 1 stop bit
  Serial1.begin(2400, SERIAL_8N1, cRXPin, cTXPin);
  nextBurst = millis();                               // initialize burst timing
}

void loop() {
  uint32_t curTime = millis();                        // get current time

  if (curTime >= nextBurst) {
    if (runState) {
      for (uint16_t i = 0; i < cNumBytes; i++) {
        Serial1.write(cTXData);
      }
      digitalWrite(cLEDPin, !digitalRead(cLEDPin)); // toggle LED
    }
    nextBurst += cBurstInterval;
  }
}