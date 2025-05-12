/*
The master board is responsible for
drive, ultrasonic, IR and communicating
the intake and sweeper needs to board #2.

Communication from the master to the secondary is
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

boolean state;


void setup() {
  pinMode(cPowerFlag, OUTPUT); 
  pinMode(cSweepFlag, OUTPUT);
  pinMode(cBucketFlag, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(cPowerFlag, 1);
  digitalWrite(cSweepFlag, 0);
  digitalWrite(cBucketFlag, 0);

  //state = !state;
  delay(1000);

  digitalWrite(cPowerFlag, 0);
  digitalWrite(cSweepFlag, 1);
  digitalWrite(cBucketFlag, 0);

  delay(1000);

  digitalWrite(cPowerFlag, 0);
  digitalWrite(cSweepFlag, 0);
  digitalWrite(cBucketFlag, 1);

  delay(1000);
}
