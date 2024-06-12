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

// These values correspond to the appropriate pins on the Teensy
#define SHOULDER_MIN 17
#define SHOULDER_MAX 18
#define ELBOW_MIN 22
#define ELBOW_MAX 19

struct Joint
{
  float angle_continuous;
  int error;
};

Joint elbow, shoulder, waist;

float elbow_zero_offset_deg = 240.0;
float shoulder_zero_offset_deg = 120.0;
float waist_zero_offset_deg = 0;

ros::NodeHandle nh;

std_msgs::Float32MultiArray arm_brushless_fb_msg;
ros::Publisher arm_brushless_fb_pub("/armBrushlessFb", &arm_brushless_fb_msg);

std_msgs::Float32MultiArray arm_brushless_limits;
ros::Publisher arm_brushless_limits_pub("/armBrushlessLimits", &arm_brushless_limits);

float arm_brushless_angle_ps[3] = {0, 0, 0};
float brushless_limit_array[4] = {};

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

void read_joint_angle_single(Joint &joint, uint8_t CS_pin)
{
  uint16_t result;
  result = getPositionSPI(CS_pin, 12);
  if (result == 0xFFFF)
  {
    joint.error = -1;
    return;
  }
  joint.angle_continuous = mapFloat((float)result, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
  joint.error = 0;
}

void setup()
{
  nh.initNode();
  nh.advertise(arm_brushless_limits_pub);
  nh.advertise(arm_brushless_fb_pub);

  while (!nh.connected())
  {
    nh.negotiateTopics();
    nh.spinOnce();
  }

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);

  last_time_ms = millis();

  resetAMT22(CS1);
  resetAMT22(CS2);
  resetAMT22(CS3);

  pinMode(SHOULDER_MIN, INPUT);
  pinMode(SHOULDER_MAX, INPUT);
  pinMode(ELBOW_MIN, INPUT);
  pinMode(ELBOW_MAX, INPUT);
}

void loop()
{
  if (millis() - last_time_ms > POLL_DELAY_MS)
  {
    // Update ROS info
    nh.spinOnce();
    delay(1);

    // Write limit switch readings to array
    brushless_limit_array[0] = !(digitalRead(SHOULDER_MIN));
    brushless_limit_array[1] = !(digitalRead(SHOULDER_MAX));
    brushless_limit_array[2] = !(digitalRead(ELBOW_MIN));
    brushless_limit_array[3] = !(digitalRead(ELBOW_MAX));

    // Read encoder angles
    read_joint_angle_single(elbow, CS1);
    read_joint_angle_single(shoulder, CS2);
    read_joint_angle_single(waist, CS3);

    // Write encoder values to array
    arm_brushless_angle_ps[0] = elbow.error == -1 ? 0 : (elbow.angle_continuous - elbow_zero_offset_deg);
    arm_brushless_angle_ps[1] = shoulder.error == -1 ? 0 : (shoulder.angle_continuous - shoulder_zero_offset_deg);
    arm_brushless_angle_ps[2] = waist.error == -1 ? 0 : (waist.angle_continuous - waist_zero_offset_deg);

    // Publish encoder readings
    arm_brushless_fb_msg.data_length = 3;
    arm_brushless_fb_msg.data = arm_brushless_angle_ps;
    arm_brushless_fb_pub.publish(&arm_brushless_fb_msg);

    // Publish limit switch readings
    arm_brushless_limits.data_length = 4;
    arm_brushless_limits.data = brushless_limit_array;
    arm_brushless_limits_pub.publish(&arm_brushless_limits);

    last_time_ms = millis();
  }
}
