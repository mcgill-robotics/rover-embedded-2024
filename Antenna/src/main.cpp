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

void antenna_set_initial_rover_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> antenna_set_initial_rover_cmd_sub("/SetInitialRoverCoordsCmd", antenna_set_initial_rover_cmd_cb);

void antenna_overide_heading_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> antenna_overide_heading_cmd_sub("/antennaHeadingOverideCmd", antenna_overide_heading_cmd_cb);

void rover_gps_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> rover_gps_cmd_sub("/roverGPSData", rover_gps_cmd_cb); // used to be /roverGPSFeedCmd


// DECLARATIONS
extern void antenna_setup();
extern void antenna_loop();
extern float rover_coords[2];
extern float antenna_heading_params[4];
extern bool isOveriden;
extern float servo_angle[1];


static const uint32_t GPSBaud = 9600;
extern float base_gps_coords[2];
extern void gps_setup();
extern void gps_loop();

extern double sin_theta;

unsigned long last_time;

void setup(){
  // ROS Setup
  nh.initNode();
  nh.advertise(antennaGPSData_pub);

  nh.subscribe(antenna_set_initial_rover_cmd_sub);
  nh.subscribe(antenna_overide_heading_cmd_sub);
  nh.subscribe(rover_gps_cmd_sub);
  nh.negotiateTopics();
  
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  Serial1.begin(GPSBaud);
  while (!Serial1);

  antenna_setup();
  // gps_setup();

  last_time = millis();
}

void loop(){
  while(millis() - last_time < CONTROL_LOOP_PERIOD_MS);
  last_time = millis();

  gps_loop();
  antenna_heading_params[0] = base_gps_coords[0];
  antenna_heading_params[1] = base_gps_coords[1];
  
  antenna_loop();
  ros_loop();
}

void ros_loop()
{
    // Publish Antenna GPS Coords
    float temp[2] = {antenna_heading_params[0],antenna_heading_params[1]};

    antennaGPSDataMsg.data_length = 2;
    antennaGPSDataMsg.data = temp;
    antennaGPSData_pub.publish(&antennaGPSDataMsg);  

    nh.spinOnce();
}

void antenna_set_initial_rover_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  antenna_heading_params[2] = input_msg.data[0];
  antenna_heading_params[3] = input_msg.data[1]; 
}

void rover_gps_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  rover_coords[0] = input_msg.data[0];
  rover_coords[1] = input_msg.data[1];    
}

void antenna_overide_heading_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  isOveriden = input_msg.data[0];
  if(isOveriden){
    servo_angle[0] = input_msg.data[1];
  }
}