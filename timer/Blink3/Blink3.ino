/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#define RED_OUT    3
#define BLUE_OUT   5
#define GREEN_OUT  6

#define LED_ON_TIME  1000
#define LED_OFF_TIME  500

void setup() {                
  // initialize the digital pin as an output.
  pinMode(BLUE_OUT, OUTPUT);
  pinMode(GREEN_OUT, OUTPUT);
  pinMode(RED_OUT, OUTPUT);
}

void loop() {
  digitalWrite(RED_OUT, HIGH);    // set the LED on
  delay(LED_ON_TIME);             // wait for a second
  digitalWrite(RED_OUT, LOW);     // set the LED off
  delay(LED_OFF_TIME);            // wait for a second

  digitalWrite(GREEN_OUT, HIGH);  // set the LED on
  delay(LED_ON_TIME);             // wait for a second
  digitalWrite(GREEN_OUT, LOW);   // set the LED off
  delay(LED_OFF_TIME);            // wait for a second

  digitalWrite(BLUE_OUT, HIGH);   // set the LED on
  delay(LED_ON_TIME);             // wait for a second
  digitalWrite(BLUE_OUT, LOW);    // set the LED off
  delay(LED_OFF_TIME);            // wait for a second
}
