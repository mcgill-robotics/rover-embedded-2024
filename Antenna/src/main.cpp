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

void antenna_overide_gps_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> antenna_overide_gps_cmd_sub("/antennaGPSOverideCmd", antenna_overide_gps_cmd_cb);

void antenna_overide_heading_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> antenna_overide_heading_cmd_sub("/antennaHeadingOverideCmd", antenna_overide_gps_cmd_cb);

void rover_gps_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> rover_gps_cmd_sub("/roverGPSFeedCmd", rover_gps_cmd_cb); // used to be /roverGPSFeedCmd


// DECLARATIONS
extern void antenna_setup();
extern void antenna_loop();
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

  nh.subscribe(antenna_overide_gps_cmd_sub);
  nh.subscribe(antenna_overide_heading_cmd_sub);
  nh.subscribe(rover_gps_cmd_sub);
  nh.negotiateTopics();
  
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  // Wait for Serial
  // Serial.begin(115200); -- removed might interveen with ros
  Serial1.begin(GPSBaud);
  while (!Serial1);

  antenna_setup();
  // gps_setup();

  last_time = millis();
}

void loop(){
  while(millis() - last_time < CONTROL_LOOP_PERIOD_MS);
  last_time = millis();

  antenna_heading_params[0] = base_gps_coords[0];
  antenna_heading_params[1] = base_gps_coords[1];

  // Serial.print(antenna_heading_params[0]);
  // Serial.print(" , ");
  // Serial.print(antenna_heading_params[1]);
  // Serial.print(" || ");
  //  Serial.print(rover_coords[0]);
  // Serial.print(" , ");
  // Serial.print(rover_coords[1]);
  // Serial.print(" || ");
  // Serial.println(sin_theta);
  
  antenna_loop();
  gps_loop();
  ros_loop();
}

void ros_loop()
{
    if(IsOveriden == 0){
      antenna_heading_params[0] = base_gps_coords[0] ;
      antenna_heading_params[1] = base_gps_coords[1] ;
    }

    // Publish Antenna GPS Coords
    float temp[2] = {antenna_heading_params[0],antenna_heading_params[1]};

    antennaGPSDataMsg.data_length = 2;
    antennaGPSDataMsg.data = temp;
    antennaGPSData_pub.publish(&antennaGPSDataMsg);  

    nh.spinOnce();
    delay(1); // Delay may require change
}

void antenna_overide_gps_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  antenna_heading_params[0] = input_msg.data[0];
  antenna_heading_params[1] = input_msg.data[1]; 
}

void antenna_overide_heading_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  //IsOveriden = true;
  IsOveriden = input_msg.data[0];
  if(IsOveriden){
    antenna_heading_params[0] = input_msg.data[1];
    antenna_heading_params[1] = input_msg.data[2]; 
  }
}


void rover_gps_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  rover_coords[0] = input_msg.data[0];
  rover_coords[1] = input_msg.data[1];    
}