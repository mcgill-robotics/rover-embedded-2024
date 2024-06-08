#include <Arduino.h>
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"

#define dirPin 0               // direction: high or low
#define stepPin 1              // steps/revolutions: run stepsPerRev for loop for # of steps to take
#define stepsPerRevolution 200 // number of steps per 1 rev
                               // speed: frequency of pulses sent to step pin, higher freq = shorter delay
ros::NodeHandle nh;
float data[1];
std_msgs::Float64MultiArray science_data;

void sci_cb(const std_msgs::Float64MultiArray &input_msg);
ros::Subscriber<std_msgs::Float64MultiArray> sci_sub("/science", sci_cb);

void sci_cb(const std_msgs::Float64MultiArray &input_msg)
{
  for (int i = 0; i < 0.25 * stepsPerRevolution; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(7000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(7000);
  }
}

void setup()
{
  nh.initNode();
  nh.subscribe(sci_sub);
  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  science_data.data_length = 1;
  science_data.data = data;

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
