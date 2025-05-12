
/*

Lab 005 - Team 001
Jahangir Abdullayev (251283871), 

The following is the code for the moving robot.
This code is intended for board #2, the slave.

*/

/*

The slave board is responsible for
getting orders from the master to 
control the sweeper and the intake bucket.

Communication from the master and slave is
via three digital pins. Board #1 uses the pins
as outputs while board #2 reads them as inputs.

Pin 1   Powers all board #2 devices (state)
  ON    powered
  OFF   power off

Pin 2   Sweeper (state)
  ON    sweeper spinning inwards
  OFF   sweeper reverse

Pin 3   Intake bucket (momentary action)
  ON    preform one lift cycle
  OFF   intake is down

*/

// Uncomment keywords to enable debugging output
#define DEBUG_MODE  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>



// Button structure
struct Button {
  const int pin;                                                       // GPIO pin for button
  volatile uint32_t numberPresses;                                     // counter for number of button presses
  uint32_t nextPressTime;                                              // time of next allowable press in milliseconds
  volatile bool pressed;                                               // flag for button press event
};

// Encoder structure
struct Encoder {
  const int PinA;                                                      // GPIO pin for encoder Channel A
  const int PinB;                                                      // GPIO pin for encoder Channel B
  volatile long pos;                                                   // current encoder position
};

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void ARDUINO_ISR_ATTR usTimerISR(void* arg);
void ARDUINO_ISR_ATTR echoISR(void* arg);

// Constants
const int cHeartbeatInterval = 75;                                     // heartbeat update interval, in milliseconds
const int cSmartLED          = 23;                                     // when DIP switch S1-4 is on, SMART LED is connected to GPIO23
const int cSmartLEDCount     = 1;                                      // number of Smart LEDs in use
const long cDebounceDelay    = 170;                                    // switch debounce delay in milliseconds
const int cNumMotors         = 1;                                      // Number of DC motors
const int cIN1Pin[]          = {26, 16};                               // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[]          = {27, 17};                               // GPIO pin(s) for IN2 for left and right motors (A, B)
const int cPWMRes            = 8;                                      // bit resolution for PWM
const int cMinPWM            = 150;                                    // PWM value for minimum speed that turns motor
const int cMaxPWM            = pow(2, cPWMRes) - 1;                    // PWM value for maximum speed
const int cPWMFreq           = 20000;                                  // frequency of PWM signal
const int cCountsRev         = 1096;                                   // encoder pulses per motor revolution
const int cPotPin            = 36;                                     // GPIO pin for drive speed potentiometer (A0)
const int cMotorEnablePin    = 39;                                     // GPIO pin for motor enable switch (DIP S1-1)
const int cTrigger           = 11;                                     // trigger duration in microseconds

const double janiksConstant         = 140.0;                                   // constant to translate encode to distance

// Variables
boolean motorsEnabled        = true;                                   // motors enabled flag
boolean timeUp3sec           = false;                                  // 3 second timer elapsed flag
boolean timeUp2sec           = false;                                  // 2 second timer elapsed flag
uint32_t lastHeartbeat       = 0;                                      // time of last heartbeat state change
uint32_t curMillis           = 0;                                      // current time, in milliseconds
uint32_t timerCount3sec      = 0;                                      // 3 second timer count in milliseconds
uint32_t timerCount2sec      = 0;                                      // 2 second timer count in milliseconds
uint32_t robotModeIndex      = 0;                                      // robot operational state                              
uint32_t lastTime            = 0;                                      // last time of motor control was updated
Button modeButton            = {0, 0, 0, false};                       // NO pushbutton PB1 on GPIO 0, low state when pressed
Encoder encoder[]            = {{35, 32, 0},                           // left encoder (A) on GPIO 35 and 32, 0 position 
                                {33, 25, 0}};                          // right encoder (B) on GPIO 33 and 25, 0 position
uint8_t driveSpeed           = 0;                                      // motor drive speed (0-255)
uint8_t driveIndex           = 0;                                      // state index for run mode

// Servo varibles
const int cServoRight        = 4;
const int cServoLeft         = 13;

// int ServoPositions[] = {
//   2000, 1950, 1900, 1850, 1800, 1750, 1700, 1650, 1600, 1550, 1500, 1450, 1400, 1350, 1300, 1250, 1200, 1150
// };

// int ServoPositionsReversed[] = {
//   1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850
// };

int ServoPositions[] = {
  1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000, 2100
};

int ServoPositionsReversed[] = {
  1850, 1800, 1750, 1700, 1650, 1600, 1550, 1500, 1450, 1400, 1350, 1300, 1250, 1200, 1150, 1100, 1050, 1000, 900
};

const int servoPosLength = 18;

// Variables to manage the sweep state
uint32_t lastServoUpdate = 0;
int servoIndex = 0;
int servoDirection = 1; // 1 = moving upward (toward higher PWM), -1 = moving downward
// Update interval for the servo sweep (in milliseconds)
const uint32_t servoUpdateInterval = 75;

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

uint32_t modeIndicator[6]    = {                                       // colours for different modes
  SmartLEDs.Color(255, 0, 0),                                          //   red - stop
  SmartLEDs.Color(0, 255, 0),                                          //   green - run
  SmartLEDs.Color(0, 0, 255),                                          //   blue - empty case
  SmartLEDs.Color(255, 255, 0),                                        //   yellow - empty case
  SmartLEDs.Color(0, 255, 255),                                        //   cyan - empty case
  SmartLEDs.Color(255, 0, 255)                                         //   magenta - empty case
};  


// Comms Varibles

const int cPowerFlag = 18;                                             // Pin communicating the power state
const int cSweepFlag = 19;                                             // Pin communicating the sweeping direction
const int cBucketFlag = 21;                                            // Pin communicating a bucket cycle


// vars to store the inputs
bool powerState = false;
bool sweepState = false;
bool bucketFlag = false;

void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(115200);                                              // Standard baud rate for ESP32 serial monitor
  #endif

  // Set up SmartLED
  SmartLEDs.begin();                                                   // initialize smart LEDs object
  SmartLEDs.clear();                                                   // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0));                  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                                          // set brightness [0-255]
  SmartLEDs.show();                                                    // update LED


  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);                         // setup INT1 GPIO PWM Channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);                         // setup INT2 GPIO PWM Channel
    pinMode(encoder[k].PinA, INPUT);                                   // configure GPIO for encoder Channel A input
    pinMode(encoder[k].PinB, INPUT);                                   // configure GPIO for encoder Channel B input
    // configure encoder to trigger interrupt with each rising edge on Channel A
    attachInterruptArg(encoder[k].PinA, encoderISR, &encoder[k], RISING);
  }

  // Set up push button
  pinMode(modeButton.pin, INPUT_PULLUP);                               // configure GPIO for mode button pin with internal pullup resistor
  attachInterruptArg(modeButton.pin, buttonISR, &modeButton, FALLING); // Configure ISR to trigger on low signal on pin
  
  pinMode(cPotPin, INPUT);                                             // set up drive speed potentiometer
  pinMode(cMotorEnablePin, INPUT);                                     // set up motor enable switch (uses external pullup)

  // Setup inputs from master board
  pinMode(cPowerFlag, INPUT_PULLDOWN); 
  pinMode(cSweepFlag, INPUT_PULLDOWN);
  pinMode(cBucketFlag, INPUT_PULLDOWN); 

  // Servo setup
  pinMode(cServoRight, OUTPUT);                                        // configure arm servo GPIO for output
  ledcAttach(cServoRight, 50, 14);                                       // setup arm servo pin for 50 Hz, 14-bit resolution
  pinMode(cServoLeft, OUTPUT);                                        // configure claw servo GPIO for output
  ledcAttach(cServoLeft, 50, 14);                                      // setup claw servo pin for 50 Hz, 14-bit resolution
  // Set initial positions for both servos
  ledcWrite(cServoRight, ServoPositions[0]);
  ledcWrite(cServoRight, ServoPositionsReversed[0]);

}

void loop() {
  uint32_t currentMillis = millis();
  long pos[] = {0, 0};                                                 // current motor positions
  int pot = 0;                                                         // raw ADC value from pot
         
  // store encoder position to avoid conflicts with ISR updates
  noInterrupts();                                                      // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
      pos[k] = encoder[k].pos;                                         // read and store current motor position
  }
  interrupts();                                                        // turn interrupts back on
 
  uint32_t curTime = micros();                                         // capture current time in microseconds
  if (curTime - lastTime > 1000) {                                     // wait 1 ms
    lastTime = curTime;                                                // update start time for next control cycle

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec += 1;                                               // increment 3 second timer count
    if (timerCount3sec > 3000) {                                       // if 3 seconds have elapsed
      timerCount3sec = 0;                                              // reset 3 second timer count
      timeUp3sec = true;                                               // indicate that 3 seconds have elapsed
    }  
   
    // 2 second timer, counts 2000 milliseconds
    timerCount2sec += 1;                                               // increment 2 second timer count
    if (timerCount2sec > 2000) {                                       // if 2 seconds have elapsed
      timerCount2sec = 0;                                              // reset 2 second timer count
      timeUp2sec = true;                                               // indicate that 2 seconds have elapsed
    }


    // check if drive motors should be powered
    motorsEnabled = !digitalRead(cMotorEnablePin);                     // if SW1-1 is on (low signal), then motors are enabled

    // update comms from master
    powerState = digitalRead(cPowerFlag);
    sweepState = digitalRead(cSweepFlag);
    bucketFlag = digitalRead(cBucketFlag);

    #ifdef DEBUG_MODE
    Serial.printf("power: %ld, sweep: %ld, bucket: %ld\n", digitalRead(cPowerFlag), digitalRead(cSweepFlag), digitalRead(cBucketFlag));
    #endif

    // only work if power pin is high
    if (powerState) {
      // if sweep is high, roll gems in
      // if low, roll reverse (this helps when intake needs to go down)
      if (sweepState) {
        setMotor(1, 255, cIN1Pin[0], cIN2Pin[0]);
      }
      else {
        setMotor(-1, 255, cIN1Pin[0], cIN2Pin[0]);
      }

      // bucket works on a momentery high
      // seeing a high, it goes to peak slowly
      if (bucketFlag) {
        // Check if it's time to update the servo positions
       if (currentMillis - lastServoUpdate >= servoUpdateInterval) {
        servoDirection = 1;
        lastServoUpdate = currentMillis;

        ledcWrite(cServoRight, ServoPositions[servoIndex]);
        ledcWrite(cServoLeft, ServoPositionsReversed[servoIndex]);

        // Update the index for the next position
        servoIndex += servoDirection;

        if (servoIndex >= servoPosLength) {
          servoIndex = servoPosLength;
        }
        }
      }
      else {
        // Check if it's time to update the servo positions
        if (currentMillis - lastServoUpdate >= servoUpdateInterval) {
          servoDirection = -1;
          lastServoUpdate = currentMillis;

          ledcWrite(cServoRight, ServoPositions[servoIndex]);
          ledcWrite(cServoLeft, ServoPositionsReversed[servoIndex]);

          // Update the index for the next position
          servoIndex += servoDirection;

          if (servoIndex <= 0) {
            servoIndex = 0;
          }
        }
      }
    }
    
    // stop all
    else {
      // stop sweeper
      setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);
      // stop servos in down position
      servoIndex = 0;
      servoDirection = 1;
      ledcWrite(cServoRight, ServoPositions[servoIndex]);
      ledcWrite(cServoLeft, ServoPositionsReversed[servoIndex]);
    }


  }

  doHeartbeat();                                                       // update heartbeat LED
}



// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                                                // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                                         // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                              // shift to the next brightness level
    if (LEDBrightnessIndex >= sizeof(LEDBrightnessLevels)) {           // if all defined levels have been used
      LEDBrightnessIndex = 0;                                          // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);         // set pixel colors to = mode 
    SmartLEDs.show();                                                  // update LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                                      // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                                                // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                                               // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure, 
// allowing multiple instances of the buttonISR to be created (1 per button)
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);                               // cast pointer to static structure

  uint32_t pressTime = millis();                                       // capture current time
  if (pressTime > s->nextPressTime) {                                  // if enough time has passed to consider a valid press
    s->numberPresses += 1;                                             // increment button press counter
    s->pressed = true;                                                 // indicate valid button press state
    s->nextPressTime = pressTime + cDebounceDelay;                     // update time for next valid press
  }  
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, 
// allowing multiple instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);                             // cast pointer to static structure
  
  int b = digitalRead(s->PinB);                                        // read state of Channel B
  if (b > 0) {                                                         // high, leading Channel A
    s->pos++;                                                          // increase position
  }
  else {                                                               // low, lagging Channel A
    s->pos--;                                                          // decrease position
  }
}

