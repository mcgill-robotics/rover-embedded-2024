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

//stepper motor
#define dirPin 9                //dirPin: set high or low for CW/CCW
#define stepPin 10              //stepPin: write HIGH then LOW for 1 step
#define stepsPerRevolution 200  //number of steps per a full revolution

void setup()  // setup for NRF
{
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
  lastTime = millis();
}

void dc_setup() { // setup for DC motors 0 and 1
  pinMode(pwm0,OUTPUT); 
  pinMode(dir0,OUTPUT); 
  pinMode(pwm1,OUTPUT); 
  pinMode(dir1,OUTPUT); 
}

 //sets the speed of DC motors
int dc_speed0 = 200; 
int dc_speed1 = 100;

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

//stepper motor
void step_setup() { //setup for stepper motor
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}
void step_cw() {  //turn CW: set dirPin HIGH
  digitalWrite(dirPin, HIGH);
}
void step_ccw() { //turn CCW: set dirPin LOW
  digitalWrite(dirPin, LOW);
}

//rotates the stepper 
void step_rotate() {   
  int revolutions = 0.25;   //revolutions sets number of revolutions to rotate(0.25 for 90 degrees)
  int delay_sec = 1000;     //controls speed: lower delay time is faster rotation
  step_cw();                //set direction: either step_cw() or step_ccw()

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
    Serial.println("Received:");
    Serial.println(rx_buffer);
    Serial.printf("time: %lu\r\n", millis());
  }
}
