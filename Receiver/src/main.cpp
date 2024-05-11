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

//DC motors(0 for screw(up/down), 1 for auger(soil colletion))
const int pwm0 = 2;   //analogWrite pwm value between 0-255 for speed
const int dir0 = 3;   //digitalWrite dir value HIGH/LOW for direction
const int pwm1 = 4;  
const int dir1 = 5;  
int screw_counter = 0;  //initial height is 0
unsigned long last_time;
bool isMotorRunning = false;
#define CONTROL_LOOP_PERIOD_MS 10
bool up = true;       //true if auger hasn't hit top yet
bool down = true;     //true if auger hasn't hit bottom yet

// //limit switch
// const int top = 11;     //top limit switch
// const int bottom = 12;  //bottom limit switch
// void ISR_top();
// void ISR_bottom();

//stepper motor
#define dirPin 1                //dirPin: digitalWrite dir value HIGH/LOW for direction
#define stepPin 0              //stepPin: write HIGH then LOW for 1 step, use a for loop for multiple steps
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
  digitalWrite(dirPin, HIGH); //fix speed


  //dc(auger)
  pinMode(pwm0,OUTPUT); 
  pinMode(dir0,OUTPUT); 
  pinMode(pwm1,OUTPUT); 
  pinMode(dir1,OUTPUT); 

  // //limit switches
  // pinMode(top, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(top), ISR_top, CHANGE);
  // pinMode(bottom, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(bottom), ISR_bottom, CHANGE);
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
  if (stepper_position == 0) { science_data_msg.data[8] = geiger_loop();}
  else if (stepper_position == 2) { science_data_msg.data[9] = geiger_loop();}
  else if (stepper_position == 4) { science_data_msg.data[10] = geiger_loop();}
  else if (stepper_position == 6) { science_data_msg.data[11] = geiger_loop();}
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
  isMotorRunning = true;
  if (up == true) {
    down = true;
    digitalWrite(dir0, HIGH);
    analogWrite(pwm0, 20);
    delay(50);
    analogWrite(pwm0, 60);
    delay(50); 
    analogWrite(pwm0, 100);
    delay(50); 
    analogWrite(pwm0, 200);
  }
}

void screw_down() { //move screw DOWN until stopped or bottom switch is hit
  isMotorRunning = true;
  if (down == true) {
    up = true;
    digitalWrite(dir0, LOW); 
    analogWrite(pwm0, 20);
    delay(50);
    analogWrite(pwm0, 60);
    delay(50); 
    analogWrite(pwm0, 100);
    delay(50); 
    analogWrite(pwm0, 200);
  }
}

void screw_stop() { //stops the screw 
  isMotorRunning = false;
  analogWrite(pwm0, 60);
  delay(500);
  analogWrite(pwm0, 20);
  delay(500); 
  analogWrite(pwm0, 0);
}

void screw_loop() { //stops screw if at top/bottom limit
  while (millis() - last_time < CONTROL_LOOP_PERIOD_MS) { 
      last_time = millis(); }

  if ( screw_counter <= -170000000) {  //lower limit: -170000000, upper limit: 31500000
    analogWrite(pwm0 ,0);
    down = false;
    up = true;
  }
  else if (screw_counter >= 31500000 ) {
    analogWrite(pwm0, 0);
    up = false;
    down = true;
  }
  else {
    if (digitalRead(dir0) == HIGH && isMotorRunning) { screw_counter ++; }      //increment counter if moving up, decrement if down
    else if (digitalRead(dir0) == LOW && isMotorRunning) { screw_counter --; }}
} 

// void ISR_top() {  //top limit switch interrupt: moves the auger down a bit and stops
//   digitalWrite(dir0, LOW);  
//   analogWrite(pwm0, 100);
//   delay(500);
//   analogWrite(pwm0, 0);
// }

// void ISR_bottom() {  //bottom limit switch interrupt: moves the auger up a bit and stops
//   digitalWrite(dir0, HIGH);  
//   analogWrite(pwm0, 100);
//   delay(500);
//   analogWrite(pwm0, 0);
// }

/////////////////////////   AUGER CONTROL(soil collection)   ////////////////////////////////////////

void auger_up() { //rotates auger CW for 15 sec
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 20);
  delay(500);
  analogWrite(pwm1, 60);
  delay(500); 
  analogWrite(pwm1, 80);
  delay(15000);
  analogWrite(pwm1, 0);
}

void auger_down() { //rotates auger CCW for 15 sec
  digitalWrite(dir1, LOW); 
  analogWrite(pwm1, 20);
  delay(500);
  analogWrite(pwm1, 60);
  delay(500); 
  analogWrite(pwm1, 80);
  delay(15000);
  analogWrite(pwm1, 0);
}

void auger_stop() { //stops the auger 
  analogWrite(pwm1, 60);
  delay(500);
  analogWrite(pwm1, 20);
  delay(500); 
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

////////////////////////////////////////////////////////////////////////////////////////////////////////
void receiveFloatArray(float *data, size_t length) {
  byte *byteData = (byte *)data;
  size_t dataSize = length * sizeof(float);
  radio.read(byteData, dataSize);
}

void loop() {
  screw_loop();
  bool rx_flag = radio.available();
  radio.startListening();
  if (rx_flag) {
    float transmitter_data[8];
    receiveFloatArray(transmitter_data, sizeof(transmitter_data) / sizeof(transmitter_data[0]));
    science_data_msg.data[0] = transmitter_data[0]; science_data_msg.data[1] = transmitter_data[1]; science_data_msg.data[2] = transmitter_data[2]; science_data_msg.data[3] = transmitter_data[3];
    science_data_msg.data[4] = transmitter_data[4]; science_data_msg.data[5] = transmitter_data[5]; science_data_msg.data[6] = transmitter_data[6]; science_data_msg.data[7] = transmitter_data[7];  
    science_pub.publish(&science_data_msg);
    nh.spinOnce();
    delay(100);
  }
}


