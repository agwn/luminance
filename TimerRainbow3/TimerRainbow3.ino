/*
 Timer Rainbow 3
 
 This example shows how to fade an LED on pin 9
 using the analogWrite() function.
 
 This example code is in the public domain.
 
 */

#include <TimerThree.h>

/* 
 * input: serial data stream of chars for each button pressed
 * output: a particular pulse for ~1ms at around 60hz 
 * licence: this code is too crap to own, have at it
 */
#define DEBUG 1
#define EN_SS 1  // enable screen saver

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// about the period of 60hz in usec
const long MAX_DUTY_CYCLE = 1024;
const long SIXTY_HZ_INTERVAL = 16643;  // uS
const long PULSE_DURATION = 2000;      // uS


//
// pins defined
//
const int ledPin = 13;

#define REDID   0
#define GREENID 1
#define BLUEID  2
#define MIN_COLOR REDID
#define MAX_COLOR BLUEID

int colorID = REDID;

//#define RED  0x01
//#define BLUE 0x02
//#define GREEN 0x04

// RED
// GREEN
// BLUE
// RED + GREEN
// RED + BLUE
// GREEN + BLUE
// RED + GREEN + BLUE

//const int colorSeqLen = 7; 
//const int colorSeq[colorSeqLen] = {
//  RED, GREEN, BLUE, RED|GREEN, RED|BLUE, GREEN|BLUE, RED|GREEN|BLUE};
//int colorSeqID = 0;

const int pwmPinCnt = 3;
const int pwmPins[pwmPinCnt] = { 
  2, 3, 5};

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
  // start serial port at 57600 bps bc thats how i roll 
  // it also aides in wireless programming with the Fio (pure sex)
  Serial.begin(57600);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  for (int i=0; i<pwmPinCnt; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }

  Timer3.initialize(SIXTY_HZ_INTERVAL); // initialize Timer3, and set a 1/60 second period
  for(int i=0; i<pwmPinCnt; i++) {
    Timer3.pwm(pwmPins[i], pwmDutyCycle[i], SIXTY_HZ_INTERVAL);
  }
  Timer3.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt
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


