//Add the SPI library so we can communicate with the CMA3000 sensor
#include <SPI.h>
#include <avr/sleep.h>

#define RELEASE_MODE 1
#define ENABLE_SLEEP 1

#define DEBUG 0
#define DEBUG_ACCEL  0
#define DEBUG_SLEEP 0

//
// pins defined
//
// wake up pins
const int wakePin = 2;

const int ledPin = 13;

#define REDID    0
#define GREENID  1
#define BLUEID   2
#define MIN_COLOR REDID
#define MAX_COLOR BLUEID

int colorID = REDID;

const int pwmPinCnt = 3;
const int pwmPins[pwmPinCnt] = { 
  6, 5, 9};

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

const unsigned long wakeDuration = 6000; // 6 seconds

int sleepStatus = 0;  // variable to store a request for sleep
unsigned long wakeTime = 0;

//Assign the Chip Select signal to pin 10.
int CS=10;

#define RANGE_2G  1

// 56 counts/G for 2G, 14 counts/G for 8G
#if RANGE_2G
#define COUNTS_PER_G  56
#else
#define COUNTS_PER_G  14
#endif

#define DELAY_TIME  20

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

typedef enum {
  sys_init = 0,
  sys_idle,
  sys_active,
  sys_sleep,
  sys_wake,
}
sys_state_t;

static sys_state_t state;

//This buffer will hold values read from the ADXL345 registers.
//char values[10];
//char output[20];
char intStatus=0;

int revID;
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;

void wakeUpNow()
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.

  // TODO check what the actual event is
  intStatus = readRegister(INT_STATUS);

  if (intStatus) {
    // FF or MD
    state = sys_wake;
  } 
  else {
    // if state = sys_shutdown
    // dont handle?
    state = sys_active;
  }  
}

void setup() {

  state = sys_init;

  pinMode(wakePin, INPUT);
  digitalWrite(wakePin, HIGH);  // enable weak pullup

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  for (int i=0; i<pwmPinCnt; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }

  wakeTime = millis();  // initalize wake time

#if RELEASE_MODE
  // disable unused things
  ADCSRA &= ~(0x80);  // disable the ADC

  // could disable the Analog Comparitor (did not see power savings)
  //ADCSRB &= ~(0x40);  // make sure the Analog Comparator is not trying to use ADC MUX
  //ACSR |= 0x80;       // disable the Analog Comparator
  //ACSR &= ~(0x40);    // make sure the AC is not using the internal ref

  // could disable digital input circuitry on ADC and AC pins
  //DIDR0 = 0x30;  // ADC5D-ADC4D (we are using A0-A3 for leds)
  //DIDR1 = 0x03;  // AIN1D-AIN0D
#endif

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

  // enable interrupt input
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
  writeRegister(CTRL, (G_RANGE_2 | INT_LEVEL_LOW | MDET_NO_EXIT | I2C_DIS | MODE_MD_10)); // Activate measurement mode: 2g/motion detect/10Hz
  //writeRegister(CTRL, (G_RANGE_2 | INT_LEVEL_LOW | I2C_DIS | MODE_40)); // Activate measurement mode: 2g/40Hz
  //writeRegister(CTRL, (G_RANGE_2 | INT_LEVEL_LOW | I2C_DIS | MODE_400)); // Activate measurement mode: 2g/400Hz
#else
  writeRegister(CTRL, (INT_LEVEL_LOW | I2C_DIS | MODE_400)); // Activate measurement mode: 8g/400Hz
#endif
  delayMicroseconds(44);

  // Dummy read to generate first INT pin Lo to Hi
  // transition in all situations, also while debugging
  x = readRegister(DOUTX);

  //Create an interrupt that will trigger when a motion event is detected.
  attachInterrupt(0, wakeUpNow, LOW);

  state = sys_idle;
}

void loop(){

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

  /*
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
   state = sys_active;
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
   delay(50);
   }
   
   intStatus=0;    
   }
   */

#if DEBUG_ACCEL
  //Print the results to the terminal.
  Serial.print("accel: ");
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);
#endif

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
  if ((millis() - wakeTime) >= wakeDuration) {
#if DEBUG_SLEEP
    Serial.println("Entering Sleep mode");
    delay(100);     // this delay is needed, the sleep
    //function will provoke a Serial error otherwise!!
#endif
#if ENABLE_SLEEP
    sleepNow();     // sleep function called here
#endif
  }

  delay(DELAY_TIME); 
}

void sleepNow()         // here we put the arduino to sleep
{
  state = sys_sleep;

  // turn off all leds
  for (int i=0; i<pwmPinCnt; i++) {
    analogWrite(pwmPins[i], 0);
  }

  // reset the accelerometer and put into motion detect mode
  //resetAccelerometer();
  writeRegister(CTRL, (G_RANGE_2 | INT_LEVEL_LOW | MDET_NO_EXIT | I2C_DIS | MODE_MD_10)); // Activate measurement mode: 2g/motion detect/10Hz

  /* Time to set the sleep mode.
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * We want as much power savings as possible, so we use
   * sleep mode: SLEEP_MODE_PWR_DOWN
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  /* Now to enable an interrupt. Do it here so if pushed when
   * running it doesn't interrupt the running program.
   *
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
   *
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */

  attachInterrupt(0,wakeUpNow, LOW);

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  // recorde the current time immediatly after wake up
  wakeTime = millis();

  sleep_disable();         // first thing after waking from sleep: disable sleep...

  detachInterrupt(0);    // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.

  state = sys_wake;

  writeRegister(CTRL, (G_RANGE_2 | INT_LEVEL_LOW | I2C_DIS | MODE_40 | INT_DIS)); // Activate measurement mode: 2g/40Hz

#if DEBUG_SLEEP
  delay(50);
  Serial.print("s: ");
  delay(100);
  Serial.println(wakeTime);
  delay(150);
#endif
}

void updateColor() {
  float tm = (float) millis() * oSpeed;

  bValue = (int)constrain((PWM_MAX_VAL*(sin(tm)/2+rOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  gValue = (int)constrain((PWM_MAX_VAL*(sin(tm+TWO_PI/3)/2+gOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
  rValue = (int)constrain((PWM_MAX_VAL*(sin(tm+2*TWO_PI/3)/2+bOffset)), PWM_MIN_VAL, PWM_MAX_VAL);
}

unsigned char resetAccelerometer()
{
  writeRegister(RSTR, 0x02);
  delayMicroseconds(44);
  writeRegister(RSTR, 0x0A);
  delayMicroseconds(44);
  writeRegister(RSTR, 0x04);

  return 0;
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










