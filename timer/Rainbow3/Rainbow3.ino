/*
 Rainbow 3
 
 This example shows how to cycle through the colors 
 of the rainbow using the analogWrite() function.
 
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

//#define RED    0x01
//#define GREEN  0x02
//#define BLUE   0x04

const int pwmPinCnt = 3;
const int pwmPins[pwmPinCnt] = { 
  3, 5, 6};

int brightness = 5;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

//
// length of pulse in period over 1024
//
#define PWM_MIN_VAL  0
#define PWM_MAX_VAL  255
#define PWM_DEF_VAL  ((int)((PWM_MAX_VAL+PWM_MIN_VAL)/2.0))
int pwmDutyCycle[pwmPinCnt] = {
  PWM_DEF_VAL, PWM_DEF_VAL, PWM_DEF_VAL};

// modulating background color
//float oSpeed = .000075;
float oSpeed = 0.00025;

const float rOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;
const float gOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;
const float bOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;

int rValue = PWM_MAX_VAL;
int gValue = PWM_MAX_VAL;
int bValue = PWM_MAX_VAL;


void setup()  {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  for (int i=0; i<pwmPinCnt; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }
} 

void loop()  { 
  // set the brightness of led pin:
  updateColor();
  analogWrite(pwmPins[REDID], rValue);
  analogWrite(pwmPins[GREENID], gValue);
  analogWrite(pwmPins[BLUEID], bValue);

#if DEBUG  
  Serial.print(rValue);
  Serial.print(", ");
  Serial.print(gValue);
  Serial.print(", ");
  Serial.println(bValue);
#endif

  // wait for 30 milliseconds to see the dimming effect    
  delay(30);                            
}

void updateColor() {
  float tm = (float) millis() * oSpeed;

  bValue = (int)constrain((PWM_MAX_VAL*(sin(tm)/2+rOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  gValue = (int)constrain((PWM_MAX_VAL*(sin(tm+TWO_PI/3)/2+gOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  rValue = (int)constrain((PWM_MAX_VAL*(sin(tm+2*TWO_PI/3)/2+bOffset)), PWM_MIN_VAL, PWM_MAX_VAL);

}


