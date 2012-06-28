/*
  Nathan Seidle
 SparkFun Electronics 2011
 
 This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Controlling an LED strip with individually controllable RGB LEDs. This stuff is awesome.
 
 The SparkFun (individually controllable) RGB strip contains a bunch of WS2801 ICs. These
 are controlled over a simple data and clock setup. The WS2801 is really cool! Each IC has its
 own internal clock so that it can do all the PWM for that specific LED for you. Each IC
 requires 24 bits of 'greyscale' data. This means you can have 256 levels of red, 256 of blue,
 and 256 levels of green for each RGB LED. REALLY granular.
 
 To control the strip, you clock in data continually. Each IC automatically passes the data onto
 the next IC. Once you pause for more than 500us, each IC 'posts' or begins to output the color data
 you just clocked in. So, clock in (24bits * 32LEDs = ) 768 bits, then pause for 500us. Then
 repeat if you wish to display something new.
 
 This example code will display bright red, green, and blue, then 'trickle' random colors down 
 the LED strip.
 
 You will need to connect 5V/Gnd from the Arduino (USB power seems to be sufficient).
 
 For the data pins, please pay attention to the arrow printed on the strip. You will need to connect to
 the end that is the begining of the arrows (data connection)--->
 
 If you have a 4-pin connection:
 Blue = 5V
 Red = SDI
 Green = CKI
 Black = GND
 
 If you have a split 5-pin connection:
 2-pin Red+Black = 5V/GND
 Green = CKI
 Red = SDI
 */

const int ledPin = 13; //On board LED

#define POS_OUT_LOGIC 0

#if POS_OUT_LOGIC
#define OUTPUT_HIGH HIGH
#define OUTPUT_LOW LOW
#else
#define OUTPUT_HIGH LOW
#define OUTPUT_LOW HIGH
#endif

#define CHAN_CNT 2
#define CHAN_0 0
#define CHAN_1 1

const int PWR[CHAN_CNT] = {
  9, 6};
const int SDI[CHAN_CNT] = {
  8, 5};
const int CKI[CHAN_CNT] = {
  7, 4};

#define STRIP_LENGTH 6
//# LEDs on this strip
long strip_colors[STRIP_LENGTH];

const int pwmPinCnt = 3;
#define PWM_MIN_VAL  0
#define PWM_MAX_VAL  64
#define PWM_DEF_VAL  ((int)((PWM_MAX_VAL+PWM_MIN_VAL)/2.0))
int pwmDutyCycle[pwmPinCnt] = {
  PWM_DEF_VAL, PWM_DEF_VAL, PWM_DEF_VAL};

// modulating background color
//float oSpeed = .000075;
const float oSpeed = 0.0025;
const int cycleDelay = 125;

const float rOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;
const float gOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;
const float bOffset = PWM_DEF_VAL/(float)PWM_MAX_VAL;

int rValue = PWM_MAX_VAL;
int gValue = PWM_MAX_VAL;
int bValue = PWM_MAX_VAL;


void setup() {
  pinMode(ledPin, OUTPUT);

  for (int i=0; i<2; i++) {
    digitalWrite(PWR[i], OUTPUT_LOW);
    pinMode(PWR[i], OUTPUT);

    digitalWrite(SDI[i], OUTPUT_LOW);
    digitalWrite(CKI[i], OUTPUT_LOW);
    pinMode(SDI[i], OUTPUT);
    pinMode(CKI[i], OUTPUT);
  }

  // shut down unused channel
  digitalWrite(PWR[CHAN_1], OUTPUT_HIGH);
  digitalWrite(SDI[CHAN_1], OUTPUT_HIGH);
  digitalWrite(CKI[CHAN_1], OUTPUT_HIGH);

  Serial.begin(57600);
  Serial.println("Hello!");

  //Clear out the array
  for(int x = 0 ; x < STRIP_LENGTH ; x++) {
    strip_colors[x] = ((long)0xff)<<((x%3)*8);
    Serial.println(strip_colors[x],HEX);
  }
  //post_frame();
  post_frame(CHAN_0);

  delay(1000);

  //randomSeed(analogRead(0));
}

void loop() {
  while(1){ //Do nothing
    //addRandom();
    updateColor();
    //loopColor();
    //post_frame(); //Push the current color frame to the strip
    post_frame(CHAN_0); //Push the current color frame to the strip

    delay(cycleDelay);                  // wait for a while
  }

  delay(1000);
}

void loopColor(void) {
  int x;
  long tmp;

  tmp = strip_colors[STRIP_LENGTH-1];

  //First, shuffle all the current colors down one spot on the strip
  for(x = (STRIP_LENGTH - 1) ; x > 0 ; x--) {
    strip_colors[x] = strip_colors[x - 1];
  }
  strip_colors[0] = tmp;
}

//Throws random colors down the strip array
void addRandom(void) {
  int x;

  //First, shuffle all the current colors down one spot on the strip
  for(x = (STRIP_LENGTH - 1) ; x > 0 ; x--) {
    strip_colors[x] = strip_colors[x - 1];
  }

  //Now form a new RGB color
  long new_color = 0;
  for(x = 0 ; x < 3 ; x++){
    new_color <<= 8;
    new_color |= random(0xFF); //Give me a number from 0 to 0xFF
    //new_color &= 0xFFFFF0; //Force the random number to just the upper brightness levels. It sort of works.
  }

  strip_colors[0] = new_color; //Add the new random color to the strip
}

void updateColor() {
  int x;
  float tm = (float) millis() * oSpeed;

  bValue = (int)constrain((PWM_MAX_VAL*(sin(tm)/2+rOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  gValue = (int)constrain((PWM_MAX_VAL*(sin(tm+TWO_PI/3)/2+gOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  rValue = (int)constrain((PWM_MAX_VAL*(sin(tm+2*TWO_PI/3)/2+bOffset)), PWM_MIN_VAL, PWM_MAX_VAL);

  //Now form a new RGB color
  long new_color = 0;
  //  for(x = 0 ; x < 3 ; x++){
  //    new_color <<= 8;
  //    new_color |= random(0xFF); //Give me a number from 0 to 0xFF
  //    //new_color &= 0xFFFFF0; //Force the random number to just the upper brightness levels. It sort of works.
  //  }
  new_color |= rValue;
  new_color <<= 8; 
  new_color |= gValue;
  new_color <<= 8; 
  new_color |= bValue;

  strip_colors[0] = new_color; //Add the new random color to the strip

  //First, shuffle all the current colors down one spot on the strip
  for(x = (STRIP_LENGTH - 1) ; x > 0 ; x--) {
    strip_colors[x] = strip_colors[x - 1];
  }

#if DEBUG  
  Serial.print(rValue);
  Serial.print(", ");
  Serial.print(gValue);
  Serial.print(", ");
  Serial.println(bValue);
#endif
}

//Takes the current strip color array and pushes it out
void post_frame (int ch) {
  //Each LED requires 24 bits of data
  //MSB: R7, R6, R5..., G7, G6..., B7, B6... B0 
  //Once the 24 bits have been delivered, the IC immediately relays these bits to its neighbor
  //Pulling the clock low for 500us or more causes the IC to post the data.

  for(int LED_number = 0 ; LED_number < STRIP_LENGTH ; LED_number++) {
    long this_led_color = strip_colors[LED_number]; //24 bits of color data

    for(byte color_bit = 23 ; color_bit != 255 ; color_bit--) {
      //Feed color bit 23 first (red data MSB)

      digitalWrite(CKI[ch], OUTPUT_LOW); //Only change data when clock is low

      long mask = 1L << color_bit;
      //The 1'L' forces the 1 to start as a 32 bit number, otherwise it defaults to 16-bit.

      if(this_led_color & mask) 
        digitalWrite(SDI[ch], OUTPUT_HIGH);
      else
        digitalWrite(SDI[ch], OUTPUT_LOW);

      digitalWrite(CKI[ch], OUTPUT_HIGH); //Data is latched when clock goes high
    }
  }

  //Pull clock low to put strip into reset/post mode
  digitalWrite(CKI[ch], OUTPUT_LOW);
  delayMicroseconds(500); //Wait for 500us to go into reset
}

