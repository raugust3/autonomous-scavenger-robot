#include <Arduino.h>

void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);

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

// Servo configuration for ESP32 using LEDC
const int cArmServo          = 4;                                      // GPIO pin for arm servo
const int cNumMotors         = 1;                                      // Number of DC motors
const int cClawServo         = 13;                                     // GPIO pin for claw servo
const long cDebounceDelay    = 170;                                    // switch debounce delay in milliseconds
const int cIN1Pin[]          = {26, 16};                               // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[]          = {27, 17};                               // GPIO pin(s) for IN2 for left and right motors (A, B)
const int cPWMRes            = 8;                                      // bit resolution for PWM
const int cMinPWM            = 150;                                    // PWM value for minimum speed that turns motor
const int cMaxPWM            = pow(2, cPWMRes) - 1;                    // PWM value for maximum speed
const int cPWMFreq           = 20000;                                  // frequency of PWM signal
const int cCountsRev         = 1096;                                   // encoder pulses per motor revolution
const int cMotorEnablePin    = 39;                                     // GPIO pin for motor enable switch (DIP S1-1)

// Variables
boolean motorsEnabled        = true;                                   // motors enabled flag
boolean timeUp4sec           = false;                                  // 3 second timer elapsed flag
boolean timeUp2sec           = false;                                  // 2 second timer elapsed flag
uint32_t lastHeartbeat       = 0;                                      // time of last heartbeat state change
uint32_t curMillis           = 0;                                      // current time, in milliseconds
uint32_t timerCount4sec      = 0;                                      // 3 second timer count in milliseconds
uint32_t timerCount2sec      = 0;                                      // 2 second timer count in milliseconds
uint32_t robotModeIndex      = 0;                                      // robot operational state                              
uint32_t lastTime            = 0;                                      // last time of motor control was updated
Button modeButton            = {0, 0, 0, false};                       // NO pushbutton PB1 on GPIO 0, low state when pressed
Encoder encoder[]            = {{35, 32, 0},                           // left encoder (A) on GPIO 35 and 32, 0 position 
                                {33, 25, 0}};                          // right encoder (B) on GPIO 33 and 25, 0 position
uint8_t driveSpeed           = 255;                                      // motor drive speed (0-255)
uint8_t driveIndex           = 0;                                      // state index for run mode
int lastPos[]                = {0, 0};                                 // Keep track of previous important encoder positions
int atPaperPos[]             = {100, 100};                                 // Encoder position before driving forward to get the ball
int sweepCount               = 0;

// keeping track of encoder counts
int encoder1;
int encoder2;
int encoder3;
int encoder4;

// PWM parameters for servos: 50Hz and 14-bit resolution
const int servoPWMFreq       = 50;    // 50Hz (typical for servos)
const int servoPWMResolution = 14;    // 14-bit resolution

// Servo positions for intake bucket
int ServoPositions[] = {
  1150, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 1950, 1900, 1850, 1800, 1750, 1700, 1650, 1600, 1550, 1500, 1450, 1400, 1350, 1300, 1250, 1200, 1150
};

int ServoPositionsReversed[] = {
  1850, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850
};

// Update interval for the servo sweep (in milliseconds)
const uint32_t servoUpdateInterval = 75;

// Variables to manage the sweep state
uint32_t lastServoUpdate = 0;
int servoIndex = 0;
int servoDirection = 1; // 1 = moving upward (toward higher PWM), -1 = moving downward

// Compute the number of positions in our lookup table
int numServoPositions = sizeof(ServoPositions) / sizeof(ServoPositions[0]);

void setup() {
  Serial.begin(115200);

  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);                         // setup INT1 GPIO PWM Channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);                         // setup INT2 GPIO PWM Channel
    pinMode(encoder[k].PinA, INPUT);                                   // configure GPIO for encoder Channel A input
    pinMode(encoder[k].PinB, INPUT);                                   // configure GPIO for encoder Channel B input
    // configure encoder to trigger interrupt with each rising edge on Channel A
    attachInterruptArg(encoder[k].PinA, encoderISR, &encoder[k], RISING);
  }

  encoder[0].pos = 0;                                                  // clear left encoder
  encoder[1].pos = 0;                                                  // clear right encoder
  
  pinMode(cArmServo, OUTPUT);                                          // configure arm servo GPIO for output
  ledcAttach(cArmServo, 50, 14);                                       // setup arm servo pin for 50 Hz, 14-bit resolution
  pinMode(cClawServo, OUTPUT);                                         // configure claw servo GPIO for output
  ledcAttach(cClawServo, 50, 14);                                      // setup claw servo pin for 50 Hz, 14-bit resolution

  // Set initial positions for both servos
  ledcWrite(cArmServo, ServoPositions[0]);
  ledcWrite(cClawServo, ServoPositionsReversed[0]);

  pinMode(cMotorEnablePin, INPUT);                                     // set up motor enable switch (uses external pullup)
}

void loop() {
  uint32_t currentMillis = millis();
  long pos[] = {0, 0};                                                 // current motor positions

  // store encoder position to avoid conflicts with ISR updates
  noInterrupts();                                                      // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
      pos[k] = encoder[k].pos;                                         // read and store current motor position
  }
  interrupts();                                                        // turn interrupts back on

  // check if drive motors should be powered
  motorsEnabled = !digitalRead(cMotorEnablePin);                     // if SW1-1 is on (low signal), then motors are enabled

  setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor forward at speed determined by pot
  setMotor(-1, driveSpeed - 8, cIN1Pin[1], cIN2Pin[1]);            // drive right motor forward at speed determined by pot, rich: pwm adjust 8
  
  // Check if it's time to update the servo positions
  if (currentMillis - lastServoUpdate >= servoUpdateInterval) {
    lastServoUpdate = currentMillis;
    
    // Update both servos with the current position value from the lookup table
    ledcWrite(cArmServo, ServoPositions[servoIndex]);
    ledcWrite(cClawServo, ServoPositionsReversed[servoIndex]);

    // Update the index for the next position
    servoIndex += servoDirection;

    // If we reach the end of the lookup table, reverse direction to create a smooth sweep
    if (servoIndex >= numServoPositions) {
      // Step back into range and reverse
      servoIndex = numServoPositions - 2;
      servoDirection = -1;
    } 
    else if (servoIndex < 0) {
      // Step forward into range and reverse
      servoIndex = 1;
      servoDirection = 1;
    }
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
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
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
