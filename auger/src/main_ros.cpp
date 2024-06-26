#include <Arduino.h>
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "common.h"
#if USE_ROS_FIRMWARE == 1
/////////////////////   ROS   //////////////////////////////
ros::NodeHandle nh;
void aug_cb(const std_msgs::Float64MultiArray &input_msg);
ros::Subscriber<std_msgs::Float64MultiArray> aug_sub("/auger", aug_cb);

void screw_up();
void screw_down();
void screw_stop();
void auger_up();
void auger_down();
void auger_stop();

////////////////////    DC motors   ////////////////////////////
const int pwm0 = 2; // screw
const int dir0 = 3;
const int pwm1 = 4; // auger
const int dir1 = 5;

// //////////////////   limit switch    //////////////////////////
const int top = 17;    // top limit switch
const int bottom = 18; // bottom limit switch
volatile unsigned long last_trigger_time_top = 0;
volatile unsigned long last_trigger_time_bottom = 0;
#define debounce_delay 100
void ISR_top();
void ISR_bottom();
int pressed = 0;
bool limit_top = false;    // true if at very top
bool limit_bottom = false; // true if at very bottom

// subscriber should receive 2 integer array. 0: stop, 1: CW, -1: CCW
// first element for screw(up/down), second for auger(soil drill)
void aug_cb(const std_msgs::Float64MultiArray &input_msg)
{
  if (input_msg.data[0] == 0)
  {
    screw_stop();
  }
  else if (input_msg.data[0] == -1)
  {
    screw_down();
  }
  else if (input_msg.data[0] == 1)
  {
    screw_up();
  }

  if (input_msg.data[1] == 0)
  {
    auger_stop();
  }
  else if (input_msg.data[1] == -1)
  {
    auger_down();
  }
  else if (input_msg.data[1] == 1)
  {
    auger_up();
  }
}

void setup()
{
  nh.initNode();
  nh.subscribe(aug_sub);
  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  // DC motor
  pinMode(pwm0, OUTPUT);
  pinMode(dir0, OUTPUT);

  // limit switch
  pinMode(top, INPUT_PULLUP);
  pinMode(bottom, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(top), ISR_top, CHANGE);
  attachInterrupt(digitalPinToInterrupt(bottom), ISR_bottom, CHANGE);
}

////////////////////////////  move up/down  ////////////////////////////
void screw_up()
{ // move screw UP until stopped or top limit switch is hit
  while (!limit_top)
  { // limit_top=false
    limit_bottom = false;
    digitalWrite(dir0, HIGH);
    analogWrite(pwm0, 40);
  }
  digitalWrite(dir0, LOW); // limit_top=true
  analogWrite(pwm0, 100);
  delay(500);
  analogWrite(pwm0, 0);
  delay(500);
}

void screw_down()
{ // move screw DOWN until stopped or bottom switch is hit
  while (!limit_bottom)
  { // limit_bottom=false
    limit_top = false;
    digitalWrite(dir0, LOW);
    analogWrite(pwm0, 40);
  }
  digitalWrite(dir0, HIGH); // limit_bottom-true
  analogWrite(pwm0, 100);
  delay(500);
  analogWrite(pwm0, 0);
  delay(500);
}

void screw_stop()
{ // stops the screw
  analogWrite(pwm0, 0);
}

////////////////////////////  auger  ////////////////////////////
void auger_down()
{
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, 100);
}

void auger_up()
{
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 100);
}

void auger_stop()
{
  analogWrite(pwm1, 0);
}

////////////////////////////  limit switches  ////////////////////////////

void ISR_bottom()
{
  unsigned long now = millis();
  if (now - last_trigger_time_bottom > debounce_delay)
  {
    last_trigger_time_bottom = now;
    if (digitalRead(bottom) == HIGH)
    { // limit switch is HIGH when touched
      limit_bottom = true;
    }
  }
}

void ISR_top()
{
  unsigned long now = millis();
  if (now - last_trigger_time_top > debounce_delay)
  {
    last_trigger_time_top = now;
    if (digitalRead(top) == HIGH)
    { // limit switch is HIGH when touched
      limit_top = true;
    }
  }
}

/////////////////////////////////////////////////////////////////////

void loop()
{
  nh.spinOnce();
  delay(10);
}
#endif // USE_ROS_FIRMWARE == 1