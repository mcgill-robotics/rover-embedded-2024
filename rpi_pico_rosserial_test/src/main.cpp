#include <Arduino.h>
#include "XL320.h"
#include "HardwareSerial.h"

// Include the ROS library
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#define PID_PERIOD_US 10000

static unsigned long lastTime;
ros::NodeHandle nh;

void servo_cmd_cb(const std_msgs::Float32MultiArray &input_msg);

std_msgs::Float32MultiArray servo_fb_msg;
std_msgs::Float32MultiArray servo_cmd_msg;
ros::Publisher servo_fb_pub("servo_fb", &servo_fb_msg);
ros::Subscriber<std_msgs::Float32MultiArray> servo_cmd_sub("servo_cmd", servo_cmd_cb);

// SERVO CONFIGURATION
XL320 robot;
char rgb[] = "rgbypcwo";
int servo_id = 16;
int led_color = 0;
float servo_setpoint_raw[1] = {0};
float servo_pos_raw[1] = {0};
float servo_pos_deg[1] = {0};
float servo_setpoint_deg[1] = {0};

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_setup()
{
  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  // Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  robot.begin(Serial1); // Hand in the serial object you're using
  robot.setJointSpeed(servo_id, 1023 / 2);
}

void servo_loop()
{
  // LED test.. let's randomly set the colour (0-7)
  robot.LED(servo_id, &rgb[random(0, 7)]);

  // SETPOINT TEST
  robot.moveJoint(servo_id, servo_setpoint_raw[0]);
  delay(100);
  //  Serial1.clear();
  byte buffer[256];
  XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer, 256));

  // Driver diagnostic
#if DEBUG_PRINT == 1
  p.toStream(SerialUSB);
#endif

  delay(100);
  // Serial1.clear();
  Serial1.flush();

  // Get state
  servo_pos_raw[0] = robot.getJointPosition(servo_id);
  servo_pos_deg[0] = map_float(servo_pos_raw[0], 0, 1023, 0, 359.99);

  delay(100);
}

void setup()
{
  // servo_setup();

  nh.initNode();

  servo_fb_msg.data = servo_pos_deg;
  servo_fb_msg.data_length = 1;
  servo_cmd_msg.data_length = 1;

  nh.advertise(servo_fb_pub);
  nh.subscribe(servo_cmd_sub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  lastTime = micros();
}

void loop()
{
  while (micros() < lastTime + PID_PERIOD_US)
    ;
  lastTime += PID_PERIOD_US;

  // servo_loop();
  servo_fb_pub.publish(&servo_fb_msg);

  nh.spinOnce();
}

void servo_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
  servo_setpoint_deg[0] = input_msg.data[0];
  servo_setpoint_raw[0] = map_float(servo_setpoint_deg[0], 0, 359.99, 0, 1023);
}
