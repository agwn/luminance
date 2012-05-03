/*
 Timer Rainbow 3
 
 This example shows how to fade an LED on pin 9
 using the analogWrite() function.
 
 This example code is in the public domain.
 
 */

//
// pins defined
//
const int ledPin = 13;

#define REDID    0
#define GREENID  1
#define BLUEID   2
#define MIN_COLOR REDID
#define MAX_COLOR BLUEID
int colorID = REDID;

const int pwmPinCnt = 3;
const int pwmPins[pwmPinCnt] = { 
  3, 5, 6};

//
// length of pulse in period over 1024
//
#define PWM_MIN_VAL  0
#define PWM_MAX_VAL  255
#define PWM_DEF_VAL  ((int)((PWM_MAX_VAL+PWM_MIN_VAL)/2.0))
int pwmDutyCycle[pwmPinCnt] = {
  PWM_DEF_VAL, PWM_DEF_VAL, PWM_DEF_VAL};

//
// incoming serial byte
//
// temp var for storing incoming serial data
// acsii chars are really nice to avoid colliding
// with the lower level transport protocols
//
//int inByte = 0;

// modulating background color
//float oSpeed = .000075;
float oSpeed = 0.00025;

const float rOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;
const float gOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;
const float bOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;

int rValue = PWM_MAX_VAL;
int gValue = PWM_MAX_VAL;
int bValue = PWM_MAX_VAL;

// do this once
void setup()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  for (int i=0; i<pwmPinCnt; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }
}


// do this everytime
void loop()
{
  // set the brightness of led pin:
  updateColor();

  // wait for 30 milliseconds to see the dimming effect    
  delay(30);                            
}

// this should be called with each pulse
void callback()
{
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}


void updateColor() {
  float tm = (float) millis() * oSpeed;

  bValue = (int)constrain((PWM_MAX_VAL*(sin(tm)/2+rOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  gValue = (int)constrain((PWM_MAX_VAL*(sin(tm+TWO_PI/3)/2+gOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  rValue = (int)constrain((PWM_MAX_VAL*(sin(tm+2*TWO_PI/3)/2+bOffset)), PWM_MIN_VAL, PWM_MAX_VAL);

  setPwmDutyCycle(REDID, rValue);
  setPwmDutyCycle(GREENID, gValue);
  setPwmDutyCycle(BLUEID, bValue);

#if DEBUG  
  Serial.print(rValue);
  Serial.print(", ");
  Serial.print(gValue);
  Serial.print(", ");
  Serial.println(bValue);
#endif
}


inline void setPwmDutyCycle(int pinID, int dutyCycle) {
  pwmDutyCycle[pinID] = dutyCycle;
  Timer3.setPwmDuty(pwmPins[pinID], dutyCycle);
  return;
}


