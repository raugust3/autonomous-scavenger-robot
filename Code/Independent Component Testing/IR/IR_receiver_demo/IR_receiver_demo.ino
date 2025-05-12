/*
 *  Lab 005 - Group 001
 *  IR Receiver Test Code
 */

 #include <Arduino.h>
 #include <Adafruit_NeoPixel.h>

const int cIRReceiver = 5;                                            // GPIO pin for signal from IR receiver

void setup() {

  // Setup IR receiver using UART2
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor
  Serial2.begin(2400, SERIAL_8N1, cIRReceiver);                        // 2400 baud rate, 8 data bits, no parity, 1 stop bit
}

void loop() {
  uint8_t receivedData;                                                // data received from IR receiver (byte)

  if (Serial2.available() > 0) {                                       // if data available
    receivedData = Serial2.read();                                     // read incoming byte
    Serial.printf("Received: %c\n", receivedData);                     // output received byte
  }
}
