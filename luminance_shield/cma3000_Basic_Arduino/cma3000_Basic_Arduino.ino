//Add the SPI library so we can communicate with the CMA3000 sensor
#include <SPI.h>

#define DEBUG_ACCEL  1

//Assign the Chip Select signal to pin 10.
int CS=10;

#define RANGE_2G  1

// 56 counts/G for 2G, 14 counts/G for 8G
#if RANGE_2G
#define COUNTS_PER_G  56
#else
#define COUNTS_PER_G  14
#endif

#define DELAY_TIME  150


// CMA3000 Registers
#define WHO_AM_I 0x00  // Identification register
#define REVID 0x01     // ASIC revision ID, fixed in metal
#define CTRL 0x02      // Configuration (por, operation modes)
#define STATUS 0x03    // Status (por, EEPROM parity)
#define RSTR 0x04      // Reset Register
#define INT_STATUS 0x05 // Interrupt status register
#define DOUTX 0x06     // X channel output data register
#define DOUTY 0x07     // Y channel output data register
#define DOUTZ 0x08     // Z channel output data register
#define MDTHR 0x09     // Motion detection threshold value register
#define MDFFTMR 0x0A   // Free fall and motion detection time register
#define FFTHR 0x0B     // Free fall threshold value register
#define I2C_ADDR 0x0C  // I2C device address
// 0Dh-19h Reserved

// Control Register setup
#define G_RANGE_2 0x80 // 2g range
#define INT_LEVEL_LOW 0x40 // INT active low
#define MDET_NO_EXIT 0x20 // Remain in motion detection mode
#define I2C_DIS 0x10 // I2C disabled
#define MODE_PD 0x00 // Power Down
#define MODE_100 0x02 // Measurement mode 100 Hz ODR
#define MODE_400 0x04 // Measurement mode 400 Hz ODR
#define MODE_40 0x06 // Measurement mode 40 Hz ODR
#define MODE_MD_10 0x08 // Motion detection mode 10 Hz ODR
#define MODE_FF_100 0x0A // Free fall detection mode 100 Hz ODR
#define MODE_FF_400 0x0C // Free fall detection mode 400 Hz ODR
#define INT_DIS 0x01 // Interrupts enabled

// Interrupt Status Register
#define FFDET  0x04
#define MDET_MSK     0x03
#define MDET_NONE    0x00
#define MDET_X_AXIS  0x01
#define MDET_Y_AXIS  0x02
#define MDET_Z_AXIS  0x03

// FUNCTION PROTOTYPES
unsigned char readRegister(unsigned char registerAddress);
unsigned char writeRegister(unsigned char registerAddress, unsigned char Data);

//This buffer will hold values read from the ADXL345 registers.
//char values[10];
//char output[20];
char intStatus=0;

int revID;
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;

void setup() {
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the CMA3000.
  SPI.setDataMode(SPI_MODE0);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(57600);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to have pull up enabled.
  digitalWrite(CS, HIGH);
  
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);

  revID = readRegister(REVID); // Read REVID register
  delayMicroseconds(44);

  // setup motion detection
  writeRegister(MDTHR, 0x10);    // 286 mg
  delayMicroseconds(44);
  writeRegister(MDFFTMR, 0x10);  // 100ms
  delayMicroseconds(44);

  // Activate measurement mode: 2g/400Hz, force SPI only comm, interrupts active low
#if RANGE_2G
  writeRegister(CTRL, (G_RANGE_2 | INT_LEVEL_LOW | I2C_DIS | MODE_400)); // Activate measurement mode: 2g/400Hz
#else
  writeRegister(CTRL, (INT_LEVEL_LOW | I2C_DIS | MODE_400)); // Activate measurement mode: 8g/400Hz
#endif
  delayMicroseconds(44);

  // Dummy read to generate first INT pin Lo to Hi
  // transition in all situations, also while debugging
  x = readRegister(DOUTX);

  //Create an interrupt that will trigger when a motion event is detected.
  attachInterrupt(0, accelEvent, FALLING);
}

void accelEvent()
{
  // TODO check what the actual event is
  intStatus = readRegister(INT_STATUS);
}

void loop(){
  unsigned char intStatus = 0;

  x = readRegister(DOUTX); // Read DOUTX register
  delayMicroseconds(44);
  y = readRegister(DOUTY); // Read DOUTY register
  delayMicroseconds(44);
  z = readRegister(DOUTZ); // Read DOUTZ register
  //delayMicroseconds(44);
  
  // convert from 2s complement
  if (0x80 & x) {
    x = x-0xff;
  }
  if (0x80 & y) {
    y = y-0xff;
  }
  if (0x80 & z) {
    z = z-0xff;
  }

  //Convert the accelerometer value to G's. 
  xg = x / (float)COUNTS_PER_G;
  yg = y / (float)COUNTS_PER_G;
  zg = z / (float)COUNTS_PER_G;

  if(intStatus > 0) {
    if(FFDET & intStatus) {
      Serial.println("FFDET");
      Serial.print(x);
      Serial.print(',');
      Serial.print(y);
      Serial.print(',');
      Serial.println(z);
    }

    if (MDET_MSK & intStatus) {
      Serial.print("MDET: ");
      intStatus &= MDET_MSK;
      if (MDET_X_AXIS == intStatus) {
        Serial.println("X");
      } 
      else if (MDET_Y_AXIS == intStatus) {
        Serial.println("Y");
      } 
      else if (MDET_Z_AXIS == intStatus) {
        Serial.println("Z");
      }     
    }
    detachInterrupt(0);
    delay(500);
    attachInterrupt(0, accelEvent, FALLING);

    intStatus=0;    
  }

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

// Write a byte to the acceleration sensor
unsigned char writeRegister(unsigned char registerAddress, unsigned char value)
{
  unsigned char result = 0;

  registerAddress <<= 2; // Address to be shifted left by 2
  registerAddress |= 0x02; // RW bit to be set

  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);

  return result;
}

// Read a byte from the acceleration sensor
unsigned char readRegister(unsigned char registerAddress)
{
  unsigned char result = 0;

  registerAddress <<= 2; // Address to be shifted left by 2 and RW bit to be reset

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(registerAddress);
  //Continue to read the data value fo the register
  result = SPI.transfer(0x00);
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);

  // Return new data from RX buffer
  return result;
}









