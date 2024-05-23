#include <Arduino.h>

#include <ros.h> 
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

// NRF
RF24 radio(10, 9);       // pins: CE, CSN
const byte address[6] = "00001";
uint32_t lastTime = 0;
float transmitter_data[8]; // data received(pH and moisture)

//DC motors
const int pwm0 = 2;   // screw
const int dir0 = 3;  
const int pwm1 = 4;   // auger
const int dir1 = 5;  

//Limit switches
const int top = 17;      //top limit switch
const int bottom = 18;   //bottom limit switch
volatile unsigned long last_trigger_time_top = 0;
volatile unsigned long last_trigger_time_bottom = 0;
#define debounce_delay 100
void ISR_top();
void ISR_bottom();
int pressed = 0;
bool limit_top = false;     // true if at very top
bool limit_bottom = false;  // true if at very bottom

//stepper motor
#define dirPin 0                //dirPin: digitalWrite dir value HIGH/LOW for direction
#define stepPin 1              //stepPin: write HIGH then LOW for 1 step, use a for loop for multiple steps
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

float science_data[12];
std_msgs::Float32MultiArray science_data_msg;                                     //publisher for science data
ros::Publisher science_pub("/science_data", &science_data_msg);

float stepper_pos[1];
std_msgs::Float32MultiArray stepper_pos_msg;                                       //publisher for stepper position
ros::Publisher stepper_pub("/stepper_position", &stepper_pos_msg);

void auger_cb(const std_msgs::Float32MultiArray &input_msg);                       //subscriber for auger command
ros::Subscriber<std_msgs:: Float32MultiArray> auger_sub("/augerCmd", auger_cb);

void stepper_cb(const std_msgs::Float32MultiArray &input_msg);                  //subscriber for carousel command
ros::Subscriber<std_msgs:: Float32MultiArray> stepper_sub("/stepperCmd", stepper_cb);


void setup() { 
  Serial.begin(9600);

  //ROS
  nh.initNode();
  nh.advertise(science_pub);    
  nh.advertise(stepper_pub);
  nh.subscribe(auger_sub);
  nh.subscribe(stepper_sub);
  nh.negotiateTopics();
  while (!nh.connected()) { nh.negotiateTopics(); }
  science_data_msg.data_length = 12;
  science_data_msg.data = science_data;
  stepper_pos_msg.data_length =1;
  stepper_pos_msg.data = stepper_pos;

  //geiger
  counts = 0;
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

  //dc(auger)
  pinMode(pwm0,OUTPUT); 
  pinMode(dir0,OUTPUT); 
  pinMode(pwm1,OUTPUT); 
  pinMode(dir1,OUTPUT); 

  // limit switch
  pinMode(top, INPUT_PULLUP);
  pinMode(bottom, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(top), ISR_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(bottom), ISR_bottom, CHANGE);
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
// position:cuvette mapping -> 0:1, 2:2, 4:3, 6:4
void update_geiger() {
  if (stepper_position == 0) { 
    science_data_msg.data[0] = geiger_loop();

    } // 3 csv
  else if (stepper_position == 2) { science_data_msg.data[1] = geiger_loop();}
  else if (stepper_position == 4) { science_data_msg.data[2] = geiger_loop();}
  else if (stepper_position == 6) { science_data_msg.data[3] = geiger_loop();} // geige,r moiste ph
}

/////////////////////////   STEPPER/CAROUSEL CONTROL   ////////////////////////////////////////

//returns the cuvette number that is in front of the geiger
void increase_position() {  //keeps track of position of stepper(0 to 7)
  if (stepper_position == 7) { stepper_position = 0; }
  else { stepper_position ++; }
}

//rotates the stepper 1/8 of a rotation
void rotate_stepper() {
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
  while (!limit_top) {          // limit_top=false
    limit_bottom = false;
    digitalWrite(dir0, HIGH);
    analogWrite(pwm0, 100);
  }
  digitalWrite(dir0, LOW);      // limit_top=true
  analogWrite(pwm0, 100);
  delay(500);
  analogWrite(pwm0, 0);
}

void screw_down() { //move screw DOWN until stopped or bottom switch is hit
  while (!limit_bottom) {       // limit_bottom=false
    limit_top = false;
    digitalWrite(dir0, LOW);
    analogWrite(pwm0, 100);
  }
  digitalWrite(dir0, HIGH);     // limit_bottom=true
  analogWrite(pwm0, 100);
  delay(500);
  analogWrite(pwm0, 0);
}

void screw_stop() { //stops the screw 
  analogWrite(pwm0, 0);
}

/////////////////////////   Limit Switches   ////////////////////////////////////////
void ISR_bottom() {
  unsigned long now = millis();
  if (now - last_trigger_time_bottom > debounce_delay) {
    last_trigger_time_bottom = now;
    if (digitalRead(bottom) == HIGH) {  //limit switch is HIGH when touched
      limit_bottom = true; 
    }
  }
}

void ISR_top() {
  unsigned long now = millis();
  if (now - last_trigger_time_top > debounce_delay) {
    last_trigger_time_top = now;
    if (digitalRead(top) == HIGH) {  //limit switch is HIGH when touched
      limit_top = true;
    }
  }
}

/////////////////////////   AUGER CONTROL(soil collection)   ////////////////////////////////////////

void auger_up() { //rotates auger CW for 15 sec
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 80);
}

void auger_down() { //rotates auger CCW for 15 sec
  digitalWrite(dir1, LOW); 
  analogWrite(pwm1, 80);
}

void auger_stop() { //stops the auger 
  analogWrite(pwm1, 0);
}

/////////////////////////   AUGER/CAROUSEL CALLBACK FUNCTIONS   ////////////////////////////////////////

void auger_cb(const std_msgs::Float32MultiArray &input_msg) {
  int screw = input_msg.data[0];
  int auger = input_msg.data[1];

  if (screw == 0) { screw_stop(); }         //stops at any height
  else if (screw == -1) { screw_down(); }   //goes all the way to bottom
  else if (screw == 1) { screw_up(); }      //goes all tge way to top

  if (auger == 0) { auger_stop(); }
  else if (auger == -1) { auger_down(); }
  else if (auger == 1) { auger_up(); }
}

//rotates the stepper by 45 degrees to position indicated by input_msg, updates geiger data, publishes the new position
void stepper_cb(const std_msgs::Float32MultiArray &input_msg) { 
  //stepper_position: integer 0-7 is position of carousel
  //starts at pos 0 with cuvette #1 at geiger
  //pos 0/2/4/6 are diagonal to scan geiger, pos 1/3/5/7 are straight for dispensing dirt to cuvette
  rotate_stepper();
  increase_position(); 
  update_geiger();
  stepper_pos_msg.data[0] = stepper_position;
  stepper_pub.publish(&stepper_pos_msg);
}

////////////////////////////  NRF Receive data  ////////////////////////////
void receiveFloatArray(float *data, size_t length) {
  byte *byteData = (byte *)data;
  size_t dataSize = length * sizeof(float);
  radio.read(byteData, dataSize);
}

void loop() {
  bool rx_flag = radio.available();
  radio.startListening();

  if (rx_flag) {
    float transmitter_data[8];
    receiveFloatArray(transmitter_data, sizeof(transmitter_data) / sizeof(transmitter_data[0]));
    science_data_msg.data[4] = transmitter_data[0]; science_data_msg.data[5] = transmitter_data[1]; science_data_msg.data[6] = transmitter_data[2]; science_data_msg.data[7] = transmitter_data[3];
    science_data_msg.data[8] = transmitter_data[4]; science_data_msg.data[9] = transmitter_data[5]; science_data_msg.data[10] = transmitter_data[6]; science_data_msg.data[11] = transmitter_data[7];  
    science_pub.publish(&science_data_msg);
    nh.spinOnce();
    delay(100);
  }
}


