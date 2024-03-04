#include <Arduino.h>
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <SPI.h>
#include <EEPROM.h>
#include "AMT22.h"     // Assuming this is your custom library for the AMT22 encoder.
#include "rover_arm.h" // Make sure this contains necessary definitions for your project.

#define MIN_ADC_VALUE 0
#define MAX_ADC_VALUE 4095
#define POLL_DELAY_MS 50

struct Joint
{
  float angle_continuous;
  int error;
};

Joint elbow, shoulder, waist;

ros::NodeHandle nh;
std_msgs::Float32MultiArray arm_brushless_fb_msg;
ros::Publisher arm_brushless_fb_pub("armBrushlessFB", &arm_brushless_fb_msg);

float arm_brushless_angle_ps[3] = {0, 0, 0};
int last_time_ms = 0;

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void read_joint_angle_multi(Joint &joint, uint8_t CS_pin)
{
  int16_t resultArr[2];
  int error = getTurnCounterSPI(resultArr, CS_pin, 12);

  if (error == -1)
  {
    joint.error = error;
  }
  else
  {
    float angle_deg = mapFloat((float)resultArr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
    float turns = resultArr[1];
    joint.angle_continuous = angle_deg + 360 * turns;
    joint.error = 0;
  }
}

float read_joint_angle_single(uint8_t CS_pin)
{
  uint16_t result;
  result = getPositionSPI(CS_pin, 12);
  return mapFloat((float)result, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
}

void ros_setup()
{
  nh.initNode();
  nh.advertise(arm_brushless_fb_pub);

  while (!nh.connected())
  {
    nh.negotiateTopics();
    nh.spinOnce();
  }
}

void ros_loop()
{
  nh.spinOnce();
  delay(1);
}

void setup()
{
  ros_setup();

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);

  last_time_ms = millis();
}

void loop()
{
  ros_loop();

  if (millis() - last_time_ms > POLL_DELAY_MS)
  {
    read_joint_angle_multi(elbow, CS1);
    read_joint_angle_multi(shoulder, CS2);
    read_joint_angle_multi(waist, CS3);
    last_time_ms = millis();
  }

  arm_brushless_angle_ps[0] = elbow.error == -1 ? 0 : elbow.angle_continuous;
  arm_brushless_angle_ps[1] = shoulder.error == -1 ? 0 : shoulder.angle_continuous;
  arm_brushless_angle_ps[2] = waist.error == -1 ? 0 : waist.angle_continuous;

  arm_brushless_fb_msg.data_length = 3;
  arm_brushless_fb_msg.data = arm_brushless_angle_ps;
  arm_brushless_fb_pub.publish(&arm_brushless_fb_msg);
}
