#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>
#include <Arduino.h>

#define DEBUG_PRINT 1

// NRF
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
uint32_t lastTime = 0;

//DC motors
const int pwm0 = 2;   // pwm: set to 0-255 for speed(255 is fastest)
const int dir0 = 3;   // dir: set to high/low for direction
const int pwm1 = 4; 
const int dir1 = 5;  
int dc_speed0 = 200;  //sets the speed of DC motors
int dc_speed1 = 100;

//stepper motor
#define dirPin 9                //dirPin: set high or low for CW/CCW
#define stepPin 10              //stepPin: write HIGH then LOW for 1 step
#define stepsPerRevolution 200  //number of steps per a full revolution
int revolutions = 0.25; //controls revolutions: revolutions is number of revolutions(0.25 for 90 degrees)
int delay_sec = 1000;   //controls speed: lower delay time is faster rotation speed  

//geiger
unsigned long counts; //variable for GM Tube events
unsigned long previousMillis; //variable for measuring time
unsigned long geiger_count;
#define LOG_PERIOD 15000 // count rate


void setup()  { 
  Serial.begin(9600);

  //geiger
  counts = 0;
  Serial.begin(9600);
  pinMode(6, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), impulse, FALLING); //define external interrupts
  Serial.println("Start counter");

  //nrf
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
  lastTime = millis();

  //stepper
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH); //HIGH for cw, LOW for ccw

  //dc
  pinMode(pwm0,OUTPUT); 
  pinMode(dir0,OUTPUT); 
  pinMode(pwm1,OUTPUT); 
  pinMode(dir1,OUTPUT); 
}

void impulse() { counts++;} //counter for geiger

unsigned long geiger_loop() { //returns geiger count
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > LOG_PERIOD) {
    previousMillis = currentMillis;
    return counts;
    counts = 0; }
}

//control DC 0
void dc_forward0() {  // rotate CCW: set dir0 to HIGH
  digitalWrite(dir0, HIGH);
  analogWrite(pwm0, dc_speed0); 
}
void dc_reverse0() {  // rotate CW: set dir0 to LOW
  digitalWrite(dir0, LOW);
  analogWrite(pwm0, dc_speed0); 
}
void dc_brake0() {  // brake: set pwm0 to 0
  analogWrite(pwm0, 0); 
}

//control DC 1
void dc_forward1() {  
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, dc_speed1);
}
void dc_reverse1() {  
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, dc_speed1);
}
void dc_brake1() {  
  analogWrite(pwm1, 0);
}


//rotates the stepper for "revolutions" amount of revolutions
void step_rotate() {   
  for (int i = 0; i < revolutions * stepsPerRevolution; i++) {  //rotate Stepper for given speed/revolutions
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec);
  }
}


void loop()
{
#if DEBUG_PRINT == 1
  if (millis() - lastTime > 1000)
  {
    Serial.printf("RX Loop %lu\r\n", millis()); // Use %lu for millis() as it returns unsigned long
    lastTime = millis();
  }
#endif
  bool rx_flag = radio.available();
  if (rx_flag)
  {
    char rx_buffer[256];
    uint8_t len = radio.getDynamicPayloadSize(); // Get dynamic payload size
    if (len > 255)
      len = 255; // Ensure len does not exceed buffer size - 1 (for null terminator)
    radio.read(rx_buffer, len);
    rx_buffer[len] = '\0'; // Null-terminate the received string

    geiger_count = geiger_loop();   
    Serial.printf("%lu, ", geiger_count); //prints geiger data
    Serial.println(rx_buffer);            //prints moisture and pH data in format: "%d, %d, %d, %d, %.2f, %.2f, %.2f, %.2f"
    Serial.printf("time: %lu\r\n", millis());
  }
}

//overall prints: 50, 100, 100, 100, 100, 5.5, 5.5, 5.5, 5.5
//first number is geiger, next 4 are moisture, last 4 are pH
