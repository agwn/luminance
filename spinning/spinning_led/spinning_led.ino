//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

#define DEBUG_ACCEL  1
#define DEBUG_LEDS   0

#define MAIN_AXIS y
#define THRESH_1  0.7
#define THRESH_2  2.1
#define THRESH_3  3.2

#define DELAY_TIME  10 //25
#define RESET_DELAY 250
#define RAINBOW_ON_DUR 2000
#define SHAKE_ON_DUR  1000
#define STROBE_ON_DUR 250

#define G_THRESH  1.9
#define ACCEL_THRESH 1.5

#define COLOR_OFFSET 10
#define THETA_SCALE 2
#define THETA_0 (PI/2)

//Assign the Chip Select signal to pin 10.
int CS=10;

//ADXL345 Register Addresses
#define	DEVID		0x00	//Device ID Register
#define THRESH_TAP	0x1D	//Tap Threshold
#define	OFSX		0x1E	//X-axis offset
#define	OFSY		0x1F	//Y-axis offset
#define	OFSZ		0x20	//Z-axis offset
#define	DURATION	0x21	//Tap Duration
#define	LATENT		0x22	//Tap latency
#define	WINDOW		0x23	//Tap window
#define	THRESH_ACT	0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT	0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	//free-fall threshold
#define	TIME_FF		0x29	//Free-Fall Time
#define	TAP_AXES	0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE		0x2C	//Data rate and power mode control
#define POWER_CTL	0x2D	//Power Control Register
#define	INT_ENABLE	0x2E	//Interrupt Enable Control
#define	INT_MAP		0x2F	//Interrupt Mapping Control
#define	INT_SOURCE	0x30	//Source of interrupts
#define	DATA_FORMAT	0x31	//Data format control
#define DATAX0		0x32	//X-Axis Data 0
#define DATAX1		0x33	//X-Axis Data 1
#define DATAY0		0x34	//Y-Axis Data 0
#define DATAY1		0x35	//Y-Axis Data 1
#define DATAZ0		0x36	//Z-Axis Data 0
#define DATAZ1		0x37	//Z-Axis Data 1
#define	FIFO_CTL	0x38	//FIFO control
#define	FIFO_STATUS	0x39	//FIFO status

//This buffer will hold values read from the ADXL345 registers.
char values[10];
char output[20];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;
char tapType=0;

#define REDID    0
#define GREENID  1
#define BLUEID   2
#define MIN_COLOR REDID
#define MAX_COLOR BLUEID

const int pwmPinCnt = 3;
const int pwmPins[pwmPinCnt] = { 
  3, 5, 6};

//
// length of pulse in period over 1024
//
#define LED_MIN_VAL  0
#define LED_MAX_VAL  0xFF
#define LED_DEF_VAL  ((int)((LED_MAX_VAL+LED_MIN_VAL)/2.0))
#define LED_DEF_LOW_VAL 0x00 //(LED_DEF_VAL>>2)

#define ACCEL_MIN_VAL  -128
#define ACCEL_MAX_VAL  127

float gBase = ACCEL_MAX_VAL;
const float GSCALE = 0.8;

// modulating background color
const float rOffset = LED_DEF_VAL/(float)LED_MAX_VAL;
const float gOffset = LED_DEF_VAL/(float)LED_MAX_VAL;
const float bOffset = LED_DEF_VAL/(float)LED_MAX_VAL;

int rValue = LED_DEF_VAL;
int gValue = LED_DEF_VAL;
int bValue = LED_DEF_VAL;


void setup(){ 
  for (int i=0; i<pwmPinCnt; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }
  delay(10);

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
  //Put the ADXL345 into +/- 8G range by writing the value 0x02 to the DATA_FORMAT register.
  //Put the ADXL345 into +/- 16G range by writing the value 0x03 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x03);
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

  //Convert the accelerometer value to G's. 
  //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  xg = x * 0.0078;
  yg = y * 0.0078;
  zg = z * 0.0078;

  updateColor();

  // light up outputs
  analogWrite(pwmPins[REDID], rValue);
  analogWrite(pwmPins[GREENID], gValue);
  analogWrite(pwmPins[BLUEID], bValue);

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

void updateColor() {

  rValue = 0;
  gValue = 0;
  bValue = 0;

  if (abs(MAIN_AXIS) < (THRESH_1*ACCEL_MAX_VAL)) {
    rValue = 0;
    gValue = 0;
    bValue = 0;
  } 
  else if (abs(MAIN_AXIS) < (THRESH_2*ACCEL_MAX_VAL)) {
    rValue = LED_MAX_VAL;
  } 
  else if (abs(MAIN_AXIS) < (THRESH_3*ACCEL_MAX_VAL)) {
    gValue = LED_MAX_VAL;
  } 
  else {
    bValue = LED_MAX_VAL;
  } 

#if DEBUG_LEDS
  Serial.print(" LEDs: ");
  Serial.print(rValue);
  Serial.print(", ");
  Serial.print(gValue);
  Serial.print(", ");
  Serial.println(bValue);
#endif
}



