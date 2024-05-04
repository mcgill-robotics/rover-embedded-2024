#include <Arduino.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <ros.h> 
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

/*
Board: Rover GPS + Pan Tilt Camera Control
ROS Topics: /roverGPSData
Related Vars: rover_gps_coords[latitude, longitude]

Functionality: Returns current Rover coordinates (every loop) over ROSSerial. Takes in yaw and pitch angles to control camera pan tilt movement.
Note: A GPS value of [0,0] should be treated as an error code
*/

// ROS
ros::NodeHandle nh;
extern float rover_gps_coords[2];
void ros_loop();
std_msgs::Float32MultiArray roverGPSDataMsg;
ros::Publisher roverGPSData_pub("/roverGPSData", &roverGPSDataMsg);
void pantilt_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
ros::Subscriber<std_msgs::Float32MultiArray> pantilt_cmd_sub("/pantiltCmd", pantilt_cmd_cb);

// DECLARATIONS
static const uint32_t GPSBaud = 9600;
extern void nrf24_setup();
extern void nrf24_loop(double *param);
extern void gps_setup();
extern void gps_loop();

extern void pantilt_setup();
extern void pantilt_loop();
extern float pitch_yaw[2];
extern bool angle_updated;

void setup()
{
  // ROS Setup
  nh.initNode();
  nh.advertise(roverGPSData_pub);
  
  // Wait for Serial
  Serial.begin(115200);
  Serial1.begin(GPSBaud);
  while (!Serial);

  pantilt_setup();
  //gps_setup(); -- Currently empty  
}

void loop()
{
  gps_loop();
  ros_loop();
  pantilt_loop();

}

void ros_loop()
{
    // Publish Rover GPS Coords
    roverGPSDataMsg.data_length = 2;
    roverGPSDataMsg.data = rover_gps_coords;
    roverGPSData_pub.publish(&roverGPSDataMsg);

    nh.spinOnce();
    delay(1); // Delay may require change
}


void pantilt_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
    pitch_yaw[0] = input_msg.data[0]; // pitch
    pitch_yaw[1] = input_msg.data[1]; // yaw

    angle_updated = true;
}
