/*

Lab 005 - Team 001
Jahangir Abdullayev (251283871), 
Matthew Linders (251296414),
Richard
Jaemin Cho (251303645)

The following is the code for the moving robot.
This code is intended for board #1, the master.

*/

/*

The master board is responsible for
drive, ultrasonic, IR and communicating
the intake and sweeper needs to board #2.

Communication from the master to the slave is
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
const int cPowerFlag          = 18;                                     // Pin communicating the power state to the second board
const int cSweepFlag          = 19;                                     // Pin communicating the sweeping direction to second board
const int cBucketFlag         = 21;                                     // Pin communicating when we need a bucket cycle


// Uncomment keywords to enable debugging output
// #define DEBUG_DRIVE_SPEED    1
// #define DEBUG_ENCODER_COUNTS  1

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

// Ultrasonic sensor structure
struct Ultrasonic {
  const int triggerPin;                                                // GPIO pin for trigger
  const int echoPin;                                                   // GPIO pin for echo
  volatile uint32_t pulseBegin = 0;                                    // time of echo pulse start in microseconds
  volatile uint32_t pulseEnd = 0;                                      // time of echo pulse end in microseconds
  volatile bool newEcho = false;                                       // new (valid) echo flag
  volatile bool timeout = false;                                       // echo timeout flag
  hw_timer_t* pTimer = NULL;                                           // pointer to timer used by trigger interrupt
};

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void initUltrasonic(Ultrasonic* us);
void ping(Ultrasonic* us);
uint32_t getEchoTime(Ultrasonic* us);
uint32_t usToCm(uint32_t us);
uint32_t usToIn(uint32_t us);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void ARDUINO_ISR_ATTR usTimerISR(void* arg);
void ARDUINO_ISR_ATTR echoISR(void* arg);

// Constants
const int cHeartbeatInterval = 75;                                     // heartbeat update interval, in milliseconds
const int cSmartLED          = 23;                                     // when DIP switch S1-4 is on, SMART LED is connected to GPIO23
const int cSmartLEDCount     = 1;                                      // number of Smart LEDs in use
const long cDebounceDelay    = 170;                                    // switch debounce delay in milliseconds
const int cNumMotors         = 2;                                      // Number of DC motors
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
const int cMaxEcho           = 15000;                                  // allowable time for valid echo in microseconds
const int cIRReceiver        = 5; 

const double janiksDistanceConstant  = 140.0;                                   // constant to translate encode to distance
const double matthewsTimeConstant = 1;                                       // Trying to make the timers time
const int justinsWheelConstant = 0;                                             // Adjsutment for left wheel power

// Variables
boolean motorsEnabled        = true;                                   // motors enabled flag
boolean timeUp3sec           = false;                                  // 3 second timer elapsed flag
boolean timeUp2sec           = false;                                  // 2 second timer elapsed flag
boolean dumpTimeUp           = false;                                  // Dump timer elapsed flag
boolean offset               = false;
uint32_t lastHeartbeat       = 0;                                      // time of last heartbeat state change
uint32_t curMillis           = 0;                                      // current time, in milliseconds
uint32_t timerCount3sec      = 0;                                      // 3 second timer count in milliseconds
uint32_t timerCount2sec      = 0;                                      // 2 second timer count in milliseconds
uint32_t dumpTimerCount      = 0;                                      // Timer for the bucket lifting in milliseconds  
uint32_t driveIndex          = 0;                                      // robot operational state                              
uint32_t lastTime            = 0;                                      // last time of motor control was updated
Button modeButton            = {0, 0, 0, false};                       // NO pushbutton PB1 on GPIO 0, low state when pressed
Encoder encoder[]            = {{35, 32, 0},                           // left encoder (A) on GPIO 35 and 32, 0 position 
                                {33, 25, 0}};                          // right encoder (B) on GPIO 33 and 25, 0 position
uint8_t driveSpeed           = 0;                                      // motor drive speed (0-255)
int iterations               = 0;                                      // number of iterations the drive system has done  
Ultrasonic ultrasonic        = {21, 22};                               // trigger on GPIO21 and echo on GPIO22  
uint32_t pulseDuration;                                                // duration of pulse read from HC-SR04
uint32_t lastPulse;                                                    // last valid pulse duration from HC-SR04
uint32_t cycles              = 0;                                      // motor cycle count, used to start US measurements 
int distance                 = 0;                                      // ultrasonic ping in terms of cm 
int wCounter                 = 0;                                      // count of W's recieved in a row

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

uint32_t modeIndicator[8]    = {                                       // colours for different modes
  SmartLEDs.Color(255, 0, 0),                                          //   red - idle
  SmartLEDs.Color(0, 255, 0),                                          //   green - case1
  SmartLEDs.Color(0, 0, 255),                                          //   blue - case2
  SmartLEDs.Color(255, 255, 0),                                        //   yellow - case3
  SmartLEDs.Color(0, 255, 255),                                        //   cyan - case4
  SmartLEDs.Color(255, 0, 255),                                        //   magenta - csae5
  SmartLEDs.Color(100, 100, 100),                                      //   white - case6
  SmartLEDs.Color(50, 255, 100)                                        //   light green - case7
};                                                                            

void setup() {
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor

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

  // Set up the communication with the secondary board
  pinMode(cPowerFlag, OUTPUT); 
  pinMode(cSweepFlag, OUTPUT);
  pinMode(cBucketFlag, OUTPUT); 

  modeButton.numberPresses = 0;
  iterations = 0;

  initUltrasonic(&ultrasonic);                                         // initialize ultrasonic sensor

  // Setup IR receiver using UART2
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor
  Serial2.begin(2400, SERIAL_8N1, cIRReceiver);                        // 2400 baud rate, 8 data bits, no parity, 1 stop bit 
}

void loop() {
  //Serial.printf("NumberPresses: %d\n", modeButton.numberPresses);
  long pos[] = {0, 0};                                                 // current motor positions
  int pot = 0;                                                         // raw ADC value from pot
  uint8_t receivedData;                                                // data received from IR receiver (byte)
         
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
    if (timerCount3sec > 3000 * matthewsTimeConstant) {                                       // if 3 seconds have elapsed
      timerCount3sec = 0;                                              // reset 3 second timer count
      timeUp3sec = true;                                               // indicate that 3 seconds have elapsed
    }  
   
    // 2 second timer, counts 2000 milliseconds
    timerCount2sec += 1;                                               // increment 2 second timer count
    if (timerCount2sec > 2000 * matthewsTimeConstant) {                                       // if 2 seconds have elapsed
      timerCount2sec = 0;                                              // reset 2 second timer count
      timeUp2sec = true;                                               // indicate that 2 seconds have elapsed
    }

    //Timer for the bucket dumping
    dumpTimerCount += 1;
    if (dumpTimerCount > 3000) {
      dumpTimerCount = 0;
      dumpTimeUp = true;
    }

    // BUTTON
    if (modeButton.pressed) {                                          // Change mode on button press
      if (modeButton.numberPresses > 1) {
        modeButton.numberPresses = 0;                                  // Go back to the idle state
      }

      iterations = 0;                                                  // Reset the entire drive process
      driveIndex = 0;                                                 

      modeButton.pressed = false;                                      // reset flag

      timerCount3sec = 0;                                              // reset 3 second timer count
      timeUp3sec = false;                                              // reset 3 second timer

      timerCount2sec = 0;                                              // reset 2 second timer count
      timeUp2sec = false;                                              // reset 2 second timer

      dumpTimerCount = 0;
      dumpTimeUp = false;

      encoder[0].pos = 0;                                              // clear left encoder
      encoder[1].pos = 0;                                              // clear right encoder

      digitalWrite(cPowerFlag, 0);                                     // Turn off the intake system
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(cMotorEnablePin);                     // if SW1-1 is on (low signal), then motors are enabled
    //Check if we're ready to run the drive system
    // This checks if the motors are enabled, causes a 3 second wait, and ensures that we've pressed the button to leave idle mode.
    if (timeUp3sec && motorsEnabled && modeButton.numberPresses > 0) {
      //Serial.printf("Drive index: %d, Timer: %u, Time up: %d\n", (int)driveIndex, timerCount2sec, timeUp2sec);
      #ifdef DEBUG_DRIVE_SPEED
                  Serial.printf("Drive Speed: Pot R1 = %d, mapped = %d\n", pot, driveSpeed);
      #endif
      #ifdef DEBUG_ENCODER_COUNTS
                  Serial.printf("Encoders: Left = %ld, Right = %ld\n", pos[0], pos[1]);
                  Serial.printf("looking at: Left = %.2f; at case %.2f\n", (double)encoder[0].pos * janiksDistanceConstant / cCountsRev, (double)driveIndex);
      #endif

      switch (driveIndex) {
        case 0: // Robot stopped
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor
          encoder[0].pos = 0;                                            // clear left encoder
          encoder[1].pos = 0;                                            // clear right encoder
          digitalWrite(cPowerFlag, 0);

          // Read pot to update drive motor speed
          pot = analogRead(cPotPin);
          driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
          
          if (timeUp2sec) {                                            // update drive state after 2 seconds
            timerCount2sec = 0;                                        // reset 2 second timer count
            timeUp2sec = false;                                        // reset 2 second timer
            digitalWrite(cPowerFlag, 1);
            digitalWrite(cSweepFlag, 1);
            digitalWrite(cBucketFlag, 0);
            driveIndex++;
          }   
          break;

        case 1: // Turn slightly to change the path of the next iterations
          if (iterations > 0) { // Only do this for 2nd iteration onward
            setMotor(-1, driveSpeed * 0.65, cIN1Pin[0], cIN2Pin[0]);                         // left motor forward
            setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                               // right motor reverse (drive forward)
          }

          if (abs(encoder[1].pos) * janiksDistanceConstant / cCountsRev  >= 130 || iterations == 0) {   // Check if left wheel has rotated enough
            setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                               // stop left motor
            setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                               // stop right motor
            driveIndex++;                                                         // next state: pause
            timerCount2sec = 0;                                                   // reset 2 second timer count
            timeUp2sec = false;                                                   // reset 2 second timer

            encoder[0].pos = 0;
            encoder[1].pos = 0;
          }
          break;

        case 2: // Drive forward — motors spin in opposite directions as they are opposed by 180 degrees
          setMotor(1, driveSpeed - justinsWheelConstant, cIN1Pin[0], cIN2Pin[0]); // left motor forward
          setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                       // right motor reverse (drive forward)

          if (abs(encoder[0].pos) * janiksDistanceConstant / cCountsRev  >= 700 + iterations * 500) {   // Check if left wheel has rotated enough
            setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor
            setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
            driveIndex++;                                                     // next state: pause
            timerCount2sec = 0;                                               // reset 2 second timer count
            timeUp2sec = false;                                               // reset 2 second timer

            encoder[0].pos = 0;
            encoder[1].pos = 0;
          }
          break;
      
        case 3: // Check IR, when iR is implemented, of course.
          // Non IR version
          if (Serial2.available() > 0) {                                       // if data available
            receivedData = Serial2.read();                                     // read incoming byte
            Serial.printf("Received: %c\n", receivedData);                   // output received byte
            if (receivedData == 'W') {
              wCounter++;
              //These might suck
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // left motor stoppped
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // right motor stopped
            } else {
              wCounter = 0;
            }
          }
          
          if (timeUp2sec) {                                                  // update drive state after 2 seconds
            setMotor(-1, driveSpeed - 10, cIN1Pin[0], cIN2Pin[0]);           // left motor reverse
            setMotor(-1, driveSpeed - 10, cIN1Pin[1], cIN2Pin[1]);           // right motor reverse (forward)
          }
          if (wCounter > 3) {
            driveIndex++;
            timerCount2sec = 0;                                            // reset 2 second timer count
            timeUp2sec = false;                                            // reset 2 second timer
          }
          break;

        case 4: // Drive backward to starting position
          setMotor(-1, driveSpeed - justinsWheelConstant, cIN1Pin[0], cIN2Pin[0]);                   // left motor reverse
          setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                                           // right motor forward (drive backwards)
          digitalWrite(cBucketFlag, 0);
       
          // start US measurement once every 100 cycles (~100 ms)
          if (cycles == 0) {
            ping(&ultrasonic);                                           // start US measurement
          }

          cycles++;
          if (cycles == 100) {                                           // 100 cycles = ~100 ms
            cycles = 0;
          }
          pulseDuration = getEchoTime(&ultrasonic);                      // check if new measurement is available

          // if valid measurement, update variable and output to serial
          if (pulseDuration > 0) {
            lastPulse = pulseDuration;
            Serial.printf("Ultrasound: %ld ms = %ld cm\n", lastPulse, usToCm(lastPulse));
            if (usToCm(lastPulse) < 5 + iterations/2) {
              dumpTimeUp = false;                                       // Reset the dump timer
              dumpTimerCount = 0;
              offset = true;
              driveIndex++;

              encoder[0].pos = 0;
              encoder[1].pos = 0;
            }
          }
          break;

        case 5: // Wait for the bucket to dump
          if (offset) {
            setMotor(-1, driveSpeed - justinsWheelConstant - 10, cIN1Pin[0], cIN2Pin[0]); // left motor forward
            setMotor(1, driveSpeed - 10, cIN1Pin[1], cIN2Pin[1]);                         // right motor reverse

            if (abs(encoder[0].pos) * janiksDistanceConstant / cCountsRev  >= 30) {  // Check if left wheel has rotated enough
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                                // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                                // stop right motor
              offset = false;                                                        // next state: pause

              encoder[0].pos = 0;
              encoder[1].pos = 0;
            }
          } else {
            setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                                  // stop left motor
            setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                                  // stop right motor
            digitalWrite(cBucketFlag, 1);
            if (dumpTimeUp) {                                                        // Give the bucket time to rise 
              dumpTimeUp = false;                                                    // Reset the dump timer
              dumpTimerCount = 0;
              driveIndex++;                                                          // Move on to the next step
            }
          }
          break;
        
        case 6: // Let the bucket drop before restarting or quitting 
          digitalWrite(cSweepFlag, 0);
          digitalWrite(cBucketFlag, 0);                               // Drop the bucket back down
          if (dumpTimeUp) {                                           // Use the dump timer again 
            dumpTimeUp = false;                                       // Reset the dump timer
            dumpTimerCount = 0;

            if(iterations > 2) {                                      // We want 4 iterations of collecting. 
              modeButton.numberPresses = 0; 
            }
            driveIndex = 0;                                           // Reset the drive system
            iterations++;                                             //Add to the count of times we've collected
          }
          break;
      }
    } else {                                                           // stop when motors are disabled
          setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                      // stop left motor
          setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                      // stop right motor
          digitalWrite(cPowerFlag, 0);                                 // Drop turn off the secondary board when idle
    }
    doHeartbeat();                                                     // update heartbeat LED
  }
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
    if (modeButton.numberPresses == 0) {
      SmartLEDs.setPixelColor(0, modeIndicator[0]);                    // set pixel colour to red
    } else {
      SmartLEDs.setPixelColor(0, modeIndicator[driveIndex+1]);         // set pixel colors to = mode
    }
     
    SmartLEDs.show();                                                  // update LED
  }
}

// initialize ultrasonic sensor I/O and associated interrupts
void initUltrasonic(Ultrasonic* us) {
  pinMode(us->triggerPin, OUTPUT);                                     // configure trigger pin for output
  pinMode(us->echoPin, INPUT);                                         // configure echo pin for input
  attachInterruptArg(us->echoPin, echoISR, us, CHANGE);                // attach interrupt to state change on echo pin
  us->pTimer = timerBegin(1000000);                                    // start timer with 1 MHz frequency
  timerAttachInterruptArg(us->pTimer, usTimerISR, us);                 // configure timer ISR
}

// start ultrasonic echo measurement by sending pulse to trigger pin
void ping(Ultrasonic* us) {
  timerRestart(us->pTimer);                                            // start timer at 0
  timerAlarm(us->pTimer, cTrigger, false, 0);                          // one-shot interrupt
  digitalWrite(us->triggerPin, HIGH);                                  // start pulse on trigger pin
  us->timeout = false;                                                 // clear echo timeout flag
}

// if new pulse received on echo pin calculate echo duration in microseconds
uint32_t getEchoTime(Ultrasonic *us) {
  if (ultrasonic.newEcho && !ultrasonic.timeout) {
    ultrasonic.newEcho = false;                                        // clear new echo flag
    // Serial.printf("begin: %ld, end: %ld\n", ultrasonic.pulseBegin, ultrasonic.pulseEnd);
    return ultrasonic.pulseEnd - ultrasonic.pulseBegin;
  }
  else {                                                               // no new echo or echo timed out
    return 0;                                    
  }
}

// convert echo time in microseconds to centimetres
uint32_t usToCm(uint32_t us) {
   return (uint32_t) (double) us * 0.01724;                            // echo time in microseconds / 29 / 2 (29 us per cm)
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

// timer interrupt service routine
// single timer (and ISR) is used to end trigger pulse (short interval), then determine echo timeout
void ARDUINO_ISR_ATTR usTimerISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);                      // cast pointer to static structure

  // very short duration before interrupt (trigger length + timer overhead)
  // end pulse on trigger pin and set new alarm for echo timeout
  if (timerRead(us->pTimer) < cTrigger + 10) {                         // timer count < trigger duration + timer overhead
    digitalWrite(us->triggerPin, LOW);                                 // end pulse on trigger pin
    timerRestart(us->pTimer);                                          // restart for timer for echo timeout timing
    timerAlarm(us->pTimer, cMaxEcho, false, 0);                        // set duration of echo timeout
  }
  // longer duration before interrupt indicates that echo has timed out
  else {                                        
    us->timeout = true;                                                // set echo timeout flag
  }
}

// echo interrupt service routine
void ARDUINO_ISR_ATTR echoISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);                      // cast pointer to static structure

  // capture time when echo pin state changes from LOW to HIGH
  if (digitalRead(us->echoPin)) {
    us->pulseBegin = micros();
  } 
  // capture time when echo pin state changes from HIGH to LOW
  // only if transistion happens before echo timeout
  else if (!us->timeout) {
    us->pulseEnd = micros();
    us->newEcho = true;                                                // set new echo flag
  }
}