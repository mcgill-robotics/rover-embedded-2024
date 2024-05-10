#include <Arduino.h>
#include <ros.h> 
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include "Servo.h"
#include <math.h>
#include <iostream>

#define CONTROL_LOOP_PERIOD_MS 10


// ROS
ros::NodeHandle nh;
float antenna_gps_coords[2];
std_msgs::Float32MultiArray antennaGPSDataMsg;
ros::Publisher antennaGPSData_pub("/antennaGPSData", &antennaGPSDataMsg);

void ros_loop();

void antenna_overide_heading_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> antenna_overide_heading_cmd_sub("/antennaHeading", antenna_overide_heading_cmd_cb);

// DECLARATIONS
extern void antenna_setup();
extern void antenna_loop();
extern void set_pos(float pos);
extern float rover_coords[2];
extern float antenna_heading_params[4];

static const uint32_t GPSBaud = 9600;
extern float base_gps_coords[2];
extern void gps_setup();
extern void gps_loop();

int IsOveriden = 1;
extern double sin_theta;
extern float servo_angle[1];

unsigned long last_time;

void setup(){
  // ROS Setup
  nh.initNode();
  nh.advertise(antennaGPSData_pub);
  nh.subscribe(antenna_overide_heading_cmd_sub);
  nh.negotiateTopics();
  
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  Serial1.begin(GPSBaud);
  while (!Serial1);

  //ROS 
  antennaGPSDataMsg.data_length = 2;
  antennaGPSDataMsg.data = base_gps_coords;

  antenna_setup();
  // gps_setup();

  last_time = millis();
}

void loop(){
  while(millis() - last_time < CONTROL_LOOP_PERIOD_MS);
  last_time = millis();

  // antenna_loop(); // loop empty
  gps_loop();
  ros_loop();
}

void ros_loop()
{
    antennaGPSData_pub.publish(&antennaGPSDataMsg);  
    nh.spinOnce();
}

void antenna_overide_heading_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  float pos = (float)input_msg.data[0];
  set_pos(pos);
}
