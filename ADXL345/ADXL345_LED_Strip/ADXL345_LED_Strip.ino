//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

#define DEBUG_ACCEL  0
#define DEBUG_LEDS   1

#define DELAY_TIME  50

#define ONE_COLOR_STRIP 0

#define COLOR_OFFSET 10
#define THETA_SCALE 2
#define THETA_0 (PI/2)

//Assign the Chip Select signal to pin 10.
const int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
const char POWER_CTL = 0x2D;	//Power Control Register
const char DATA_FORMAT = 0x31;
const char DATAX0 = 0x32;	//X-Axis Data 0
const char DATAX1 = 0x33;	//X-Axis Data 1
const char DATAY0 = 0x34;	//Y-Axis Data 0
const char DATAY1 = 0x35;	//Y-Axis Data 1
const char DATAZ0 = 0x36;	//Z-Axis Data 0
const char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

const int SDI = 8; //Red wire (not the red 5V wire!)
const int CKI = 7; //Green wire

#define STRIP_LENGTH 32 //32 LEDs on this strip
//# LEDs on this strip
long strip_colors[STRIP_LENGTH];

//
// length of pulse in period over 1024
//
#define LED_MIN_VAL  0
#define LED_MAX_VAL  0xFF
#define LED_DEF_VAL  ((int)((LED_MAX_VAL+LED_MIN_VAL)/2.0))

#define ACCEL_MIN_VAL  -128
#define ACCEL_MAX_VAL  127

float gBase = ACCEL_MAX_VAL;
const float GSCALE=0.8;

// modulating background color
//float oSpeed = 0.000075;
float oSpeed = 0.0001;

// modulating background color
const float rOffset = LED_DEF_VAL/(float)LED_MAX_VAL;
const float gOffset = LED_DEF_VAL/(float)LED_MAX_VAL;
const float bOffset = LED_DEF_VAL/(float)LED_MAX_VAL;

int rValue = LED_DEF_VAL;
int gValue = LED_DEF_VAL;
int bValue = LED_DEF_VAL;


void setup(){
  pinMode(SDI, OUTPUT);
  pinMode(CKI, OUTPUT);

  //Pre-fill the color array with known values
  for(int i = 0 ; i < STRIP_LENGTH ; i++) {
    strip_colors[i] = ((long)0xFF)<< ((i%3)*8);
  }
  post_frame(); //Push the current color frame to the strip

  delay(100);

  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(57600);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
}

void loop(){

  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits).
  // To get the full value, two bytes must be combied for each axis.
  //The X value is stored in values[0] and values[1].
  x = (0xFF00&((int)values[1]<<8))|(0xff&(int)values[0]);
  //The Y value is stored in values[2] and values[3].
  y = (0xFF00&((int)values[3]<<8))|(0x00ff&(int)values[2]);
  //The Z value is stored in values[4] and values[5].
  z = (0xFF00&((int)values[5]<<8))|(0xff&(int)values[4]);

  //gBase = GSCALE*gBase + (1-GSCALE)*sqrt(x*x+y*y+z*z);
  //Serial.println(gBase);

  updateColor();
  post_frame(); //Push the current color frame to the strip

#if DEBUG_ACCEL
  //Print the results to the terminal.
  Serial.print("accel: ");
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);
#endif

  delay(DELAY_TIME);
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}


//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}


static inline void rainbowColor() {
  float t = (float) millis() * oSpeed;

  bValue = (int)constrain((LED_MAX_VAL*(sin(t)/2+rOffset)), LED_MIN_VAL, LED_MAX_VAL);
  gValue = (int)constrain((LED_MAX_VAL*(sin(t+TWO_PI/3)/2+gOffset)), LED_MIN_VAL, LED_MAX_VAL);
  rValue = (int)constrain((LED_MAX_VAL*(sin(t+2*TWO_PI/3)/2+bOffset)), LED_MIN_VAL, LED_MAX_VAL);
}

static inline void rainbowColorMove() {
//  float g = sqrt(x*x+y*y+z*z);
  float t = (float) millis() * oSpeed;

//  Serial.print(g);
//  Serial.print(" - ");
//  Serial.print(gBase);
//  Serial.print(": ");
//  Serial.println(g-gBase);

  if ((abs(x)+abs(y)+abs(z)) > 1.75*ACCEL_MAX_VAL/*abs(g - gBase) > (0.15*ACCEL_MAX_VAL)*/) {
    bValue = (int)constrain((LED_MAX_VAL*(sin(t)/2+rOffset)), LED_MIN_VAL, LED_MAX_VAL);
    gValue = (int)constrain((LED_MAX_VAL*(sin(t+TWO_PI/3)/2+gOffset)), LED_MIN_VAL, LED_MAX_VAL);
    rValue = (int)constrain((LED_MAX_VAL*(sin(t+2*TWO_PI/3)/2+bOffset)), LED_MIN_VAL, LED_MAX_VAL);
  } 
  else {
    bValue = gValue = rValue = LED_DEF_VAL;
  }
}

static inline void colorRoll() {
  float g = sqrt(x*x+y*y+z*z);
  float t = THETA_SCALE*acos(x/g)+THETA_0;

  bValue = (int)constrain((LED_MAX_VAL*(sin(t)/2+rOffset))-COLOR_OFFSET, LED_MIN_VAL, LED_MAX_VAL);
  gValue = (int)constrain((LED_MAX_VAL*(sin(t+TWO_PI/3)/2+gOffset))-COLOR_OFFSET, LED_MIN_VAL, LED_MAX_VAL);
  rValue = (int)constrain((LED_MAX_VAL*(sin(t+2*TWO_PI/3)/2+bOffset))-COLOR_OFFSET, LED_MIN_VAL, LED_MAX_VAL);
}

static inline void thresholdColor() {

  if ((abs(x) > (3.0*ACCEL_MAX_VAL)) || (abs(y) > (3.0*ACCEL_MAX_VAL)) || (abs(z) > (3.0*ACCEL_MAX_VAL))) {
    bValue = gValue = rValue = 0x00;
  } 
  else {
    rValue = (int)constrain(abs(x), LED_MIN_VAL, LED_MAX_VAL);
    gValue = (int)constrain(abs(y), LED_MIN_VAL, LED_MAX_VAL);
    bValue = (int)constrain(abs(z), LED_MIN_VAL, LED_MAX_VAL);

    //    if (abs(x) > (2.0*ACCEL_MAX_VAL)) {
    //      bValue = LED_MAX_VAL;
    //    }
    //    if (abs(y) > (2.0*ACCEL_MAX_VAL)) {
    //      gValue = LED_MAX_VAL;
    //    }
    //    if (abs(z) > (2.0*ACCEL_MAX_VAL)) {
    //      rValue = LED_MAX_VAL;
    //    }
  }
}

static inline void rectifyColor() {
  bValue = gValue = rValue = 0x00;

  if (x > 0) {
    rValue = (int)constrain(x, LED_MIN_VAL, LED_MAX_VAL);
  }
  if (y > 0) {
    gValue = (int)constrain(y, LED_MIN_VAL, LED_MAX_VAL);
  }
  if (z > 0) {
    bValue = (int)constrain(z, LED_MIN_VAL, LED_MAX_VAL);
  }
}

static inline void colorGravity() {
  rValue = (int)constrain(abs(x)-COLOR_OFFSET, LED_MIN_VAL, LED_MAX_VAL);
  gValue = (int)constrain(abs(y)-COLOR_OFFSET, LED_MIN_VAL, LED_MAX_VAL);
  bValue = (int)constrain(abs(z)-COLOR_OFFSET, LED_MIN_VAL, LED_MAX_VAL);
  //rValue = (int)constrain(x + abs(ACCEL_MIN_VAL), LED_MIN_VAL, LED_MAX_VAL);
  //gValue = (int)constrain(y + abs(ACCEL_MIN_VAL), LED_MIN_VAL, LED_MAX_VAL);
  //bValue = (int)constrain(z + abs(ACCEL_MIN_VAL), LED_MIN_VAL, LED_MAX_VAL);
  //rValue = (int)constrain(map(x, ACCEL_MIN_VAL, ACCEL_MAX_VAL, LED_MIN_VAL, LED_MAX_VAL), LED_MIN_VAL, LED_MAX_VAL);
  //gValue = (int)constrain(map(y, ACCEL_MIN_VAL, ACCEL_MAX_VAL, LED_MIN_VAL, LED_MAX_VAL), LED_MIN_VAL, LED_MAX_VAL);
  //bValue = (int)constrain(map(z, ACCEL_MIN_VAL, ACCEL_MAX_VAL, LED_MIN_VAL, LED_MAX_VAL), LED_MIN_VAL, LED_MAX_VAL);
}


void updateColor() {
  int i;

  //rainbowColor();
  //rainbowColorMove();
  colorRoll();
  //thresholdColor();
  //rectifyColor();
  //colorGravity();

  //Now form a new RGB color
  long new_color = 0;

  new_color |= rValue;
  new_color <<= 8; 
  new_color |= gValue;
  new_color <<= 8; 
  new_color |= bValue;

  strip_colors[0] = new_color; //Add the new random color to the strip

#if ONE_COLOR_STRIP
  for(i = 0; i < STRIP_LENGTH; i++) {
    strip_colors[i] = new_color;
  }
#else
  //First, shuffle all the current colors down one spot on the strip
  for(i = (STRIP_LENGTH - 1) ; i > 0 ; i--) {
    strip_colors[i] = strip_colors[i - 1];
  }
#endif

#if DEBUG_LEDS
  Serial.print(" LEDs: ");
  Serial.print(rValue);
  Serial.print(", ");
  Serial.print(gValue);
  Serial.print(", ");
  Serial.println(bValue);
#endif
}


//Takes the current strip color array and pushes it out
void post_frame (void) {
  //Each LED requires 24 bits of data
  //MSB: R7, R6, R5..., G7, G6..., B7, B6... B0 
  //Once the 24 bits have been delivered, the IC immediately relays these bits to its neighbor
  //Pulling the clock low for 500us or more causes the IC to post the data.

  for(int LED_number = 0 ; LED_number < STRIP_LENGTH ; LED_number++) {
    long this_led_color = strip_colors[LED_number]; //24 bits of color data

    for(byte color_bit = 23 ; color_bit != 255 ; color_bit--) {
      //Feed color bit 23 first (red data MSB)
      digitalWrite(CKI, LOW); //Only change data when clock is low

      long mask = 1L << color_bit;
      //The 1'L' forces the 1 to start as a 32 bit number, otherwise it defaults to 16-bit.

      if(this_led_color & mask) { 
        digitalWrite(SDI, HIGH);
      } 
      else {
        digitalWrite(SDI, LOW);
      }

      digitalWrite(CKI, HIGH); //Data is latched when clock goes high
    }
  }

  //Pull clock low to put strip into reset/post mode
  digitalWrite(CKI, LOW);
  delayMicroseconds(500); //Wait for 500us to go into reset
}








