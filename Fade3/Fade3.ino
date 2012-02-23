/*
 Fade 3
 
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

//#define RED    0x01
//#define GREEN  0x02
//#define BLUE   0x04

const int pwmPinCnt = 3;
const int pwmPins[pwmPinCnt] = { 
  3, 5, 6};

int brightness = 5;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by


void setup()  {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  for (int i=0; i<pwmPinCnt; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }
} 

void loop()  { 
  // set the brightness of led pin:
  analogWrite(pwmPins[colorID], brightness);
  
  if (0 == brightness) {
    colorID = ((colorID+1) > MAX_COLOR) ? MIN_COLOR : (colorID+1);
  }

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness == 0 || brightness == 255) {
    fadeAmount = -fadeAmount ; 
  }
  // wait for 15 milliseconds to see the dimming effect    
  delay(30);                            
}

