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
//void arm_brushed_cmd_cb(const std_msgs::Float32MultiArray &input_msg); // ignore, will be repurposed for pan tilt
std_msgs::Float32MultiArray roverGPSDataMsg;
ros::Publisher roverGPSData_pub("/roverGPSData", &roverGPSDataMsg);

// DECLARATIONS
static const uint32_t GPSBaud = 9600;
extern void nrf24_setup();
extern void nrf24_loop(double *param);
extern void gps_setup();
extern void gps_loop();

void setup()
{
  // ROS Setup
  nh.initNode();
  nh.advertise(roverGPSData_pub);
  
  // Wait for Serial
  Serial.begin(115200);
  Serial1.begin(GPSBaud);
  while (!Serial);

  //gps_setup(); -- Currently empty
  
  //nrf24_setup(); - this will be swapped with pan tilt script setup()
}

void loop()
{
  gps_loop();
  ros_loop();

  //nrf24_loop(gps_loop()); - this will be swapped with pan tilt script loop()
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

/*  ----------- To use for the CAMERA PAN TILT COMMANDS ----------------------

void arm_brushed_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
    // 0 is WP, 1 is WR, 2 is EE
    arm_brushed_setpoint_ps[0] = input_msg.data[2];
    arm_brushed_setpoint_ps[1] = input_msg.data[1];
    arm_brushed_setpoint_ps[2] = input_msg.data[0];
    mot1.set_target_angle_ps(arm_brushed_setpoint_ps[0]);
    mot2.set_target_angle_ps(arm_brushed_setpoint_ps[1]);
    HWSERIAL.printf("WP: %8.4f, WR: %8.4f, EE: %8.4f\n", arm_brushed_setpoint_ps[0], arm_brushed_setpoint_ps[1], arm_brushed_setpoint_ps[2]);

    // Motor 3 is controlled like a forklift, only up and down, range -1 to 1
    // mot3.set_target_angle_ps(arm_brushed_setpoint_ps[2]);
    mot3.move_manual(arm_brushed_setpoint_ps[2]);
}
*/