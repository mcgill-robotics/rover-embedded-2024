#include <Arduino.h>
#include <ros.h> 
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

// NRF
RF24 radio(7, 8);       // pins: CE, CSN
const byte address[6] = "00001";
uint32_t lastTime = 0;
float transmitter_data[8]; // data received(pH and moisture)

//DC motors(0 for screw(up/down), 1 for auger(soil colletion))
const int pwm0 = 2;   //analogWrite pwm value between 0-255 for speed
const int dir0 = 3;   //digitalWrite dir value HIGH/LOW for direction
const int pwm1 = 4;  
const int dir1 = 5;  

//limit switch
const int top = 11;     //top limit switch
const int bottom = 12;  //bottom limit switch
void ISR_top();
void ISR_bottom();

//stepper motor
#define dirPin 9                //dirPin: digitalWrite dir value HIGH/LOW for direction
#define stepPin 10              //stepPin: write HIGH then LOW for 1 step, use a for loop for multiple steps
#define stepsPerRevolution 200  //number of steps per a full revolution
int stepper_position = 0;       //the carousel position(even numbers are straight, odd are diagonal)

//geiger
#define LOG_PERIOD 15000        
unsigned long counts;           
unsigned long previousMillis;   
unsigned long geiger_count;
void impulse() {counts++;}      

//ROS
ros::NodeHandle nh;
std_msgs::Float64MultiArray science_data[12];
ros::Publisher science_pub("science_data", science_data);
//TODO: make motor control CB functions

void setup() { 
  Serial.begin(9600);

  //geiger
  counts = 0;
  Serial.begin(9600);
  pinMode(6, INPUT);
  attachInterrupt(digitalPinToInterrupt(6), impulse, FALLING); //define external interrupts
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

  //dc(auger)
  pinMode(pwm0,OUTPUT); 
  pinMode(dir0,OUTPUT); 
  pinMode(pwm1,OUTPUT); 
  pinMode(dir1,OUTPUT); 

  //limit switches
  pinMode(top, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(top), ISR_top, CHANGE);
  pinMode(bottom, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(bottom), ISR_bottom, CHANGE);

  //ROS
  nh.initNode();
  nh.advertise(science_pub);
}

/////////////////////////   GEIGER CONTROL   ////////////////////////////////////////

// returns geiger count of current cuvette
unsigned long geiger_loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > LOG_PERIOD) {
    previousMillis = currentMillis;
    unsigned long result = counts;
    counts = 0;
    return result;
  }
  return 0; 
}

// updates the geiger data of the cuvette in front of the geiger sensor
void update_geiger() {
  int pos = position();
  if (pos != 4) {
    science_data->data[pos + 8] = geiger_loop();
  }
}

/////////////////////////   STEPPER/CAROUSEL CONTROL   ////////////////////////////////////////

//returns the cuvette number that is in front of the geiger
int position() {
  int pos = stepper_position % 8;
  if (pos == 1) { return 0; }       //position 1 has cuvette #0 at geiger
  else if (pos == 3) { return 1; }  //position 3 has cuvette #1 at geiger
  else if (pos == 5) { return 2; }  //position 5 has cuvette #2 at geiger
  else if (pos == 7) { return 3; }  //position 7 has cuvette #3 at geiger
  else { return 4;}                 //any other position(resting position) return 4
}

//rotates the stepper by 45 degrees in a set direction
void stepper_cb(const std_msgs::String& dir) { 
  stepper_position = stepper_position + 1;
  digitalWrite(dirPin, LOW); 

  for (int i = 0; i < 0.125 * stepsPerRevolution; i++) {  
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(7000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(7000);
  }

}

/////////////////////////   SCREW CONTROL(up/down)   ////////////////////////////////////////
void screw_up() { //move screw UP until stopped or top limit switch is hit
  digitalWrite(dir0, HIGH);
  analogWrite(pwm0, 100);
}

void screw_down() { //move screw DOWN until stopped or bottom switch is hit
  digitalWrite(dir0, LOW); 
  analogWrite(pwm0, 100);
}

void screw_stop() { //stops the screw 
  analogWrite(pwm0, 0);
}

void ISR_top() {  //top limit switch interrupt: moves the auger down a bit and stops
  digitalWrite(dir0, LOW);  
  analogWrite(pwm0, 100);
  delay(500);
  analogWrite(pwm0, 0);
}

void ISR_bottom() {  //bottom limit switch interrupt: moves the auger up a bit and stops
  digitalWrite(dir0, HIGH);  
  analogWrite(pwm0, 100);
  delay(500);
  analogWrite(pwm0, 0);
}

/////////////////////////   AUGER CONTROL(soil collection)   ////////////////////////////////////////
void auger_up() { //rotates auger CW until stopped
  digitalWrite(dir0, HIGH);
  analogWrite(pwm0, 80);
}

void auger_down() { //rotates auger CCW until stopped
  digitalWrite(dir0, LOW); 
  analogWrite(pwm0, 80);
}

void auger_stop() { //stops the auger 
  analogWrite(pwm0, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  bool rx_flag = radio.available();
  if (rx_flag) {
    float transmitter_data[8];
    radio.read(transmitter_data, sizeof(transmitter_data)); //read data received from transmitter
    update_geiger(); //updates the value of science_data with geiger count
    science_data->data[0] = transmitter_data[0]; science_data->data[1] = transmitter_data[1]; science_data->data[2] = transmitter_data[2]; science_data->data[3] = transmitter_data[3];
    science_data->data[4] = transmitter_data[4]; science_data->data[5] = transmitter_data[5]; science_data->data[6] = transmitter_data[6]; science_data->data[7] = transmitter_data[7];  
    science_pub.publish(science_data);
    nh.spinOnce();
    delay(1000);
  }
}


