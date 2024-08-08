#include "common.h"

#if USE_ROS_FIRMWARE == 1
#include <Arduino.h>
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

//-------------------  ROS   ---------------------
ros::NodeHandle nh;
void stepper_cb(const std_msgs::Float64MultiArray &input_msg);
ros::Subscriber<std_msgs::Float64MultiArray> stepper_sub("/stepperCmd", stepper_cb);
void aug_cb(const std_msgs::Float64MultiArray &input_msg);
ros::Subscriber<std_msgs::Float64MultiArray> aug_sub("/augerCmd", aug_cb);
std_msgs::String debug_msg;
ros::Publisher debug_pub("debug_topic_auger", &debug_msg);
char message_buffer[256];

float science_data[12];
std_msgs::Float64MultiArray science_data_msg;
ros::Publisher science_pub("/science_data", &science_data_msg);

void screw_up();
void screw_down();
void screw_stop();
void auger_up();
void auger_down();
void auger_stop();

//-------------------  CONFIG   ---------------------
// steps/revolutions: run stepsPerRev for loop for # of steps to take
// number of steps per 1 rev
const int stepsPerRevolution = 200;
const int STEPPER_STEP_PIN = 1;
const int STEPPER_DIR_PIN = 0;
const int STEPPER_DELAY = 7000;

// screw
const int pwm0 = 2;
const int dir0 = 3;

// auger
const int pwm1 = 4;
const int dir1 = 5;

// NRF
RF24 radio(10, 9); // pins: CE, CSN
const byte address[6] = "00001";
uint32_t lastTime = 0;
float transmitter_data[8]; // data received(pH and moisture)

//-------------------   limit switch    ---------------------
#define debounce_delay 100
const int top_limit_switch_pin = 17;    // top limit switch
const int bottom_limit_switch_pin = 18; // bottom limit switch
volatile unsigned long last_trigger_time_top = 0;
volatile unsigned long last_trigger_time_bottom = 0;

float stepper_current_angle = 0.0;

volatile bool top_limit_switch_pressed = false;    // true if at very top
volatile bool bottom_limit_switch_pressed = false; // true if at very bottom

void ISR_top();
void ISR_bottom();

static void ros_printf(const char *format, ...)
{
  // Initialize the variable argument list
  va_list args;
  va_start(args, format);

  // Format the message and store it in message_buffer
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);

  // End the variable argument list
  va_end(args);

  // Assign the formatted message to debug_msg.data
  debug_msg.data = message_buffer;

  // Publish the debug message
  debug_pub.publish(&debug_msg);

  // Spin once to handle callbacks
  nh.spinOnce();
}

// subscriber should receive 2 integer array. 0: stop, 1: CW, -1: CCW
// first element for screw(up/down), second for auger(soil drill)
void aug_cb(const std_msgs::Float64MultiArray &input_msg)
{
  // Screw is the linear actuator
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

  // Auger is the drill
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

// Every publish to /science will rotate the stepper motor 0.25 revolutions
void stepper_cb(const std_msgs::Float64MultiArray &input_msg)
{
  // Calculate the total number of steps required for 0.25 revolutions
  int totalSteps = static_cast<int>(0.25 * stepsPerRevolution);

  // Loop through each step
  for (int i = 0; i < totalSteps; i++)
  {
    // Step the motor
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_DELAY);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(STEPPER_DELAY);

    // Increment the current angle based on the step
    stepper_current_angle += 360.0f / static_cast<float>(stepsPerRevolution);

    // Ensure the angle stays within the 0-360 range
    stepper_current_angle = fmod(stepper_current_angle, 360.0f);
  }
}

//-------------------  DC motor functions  ---------------------
// move screw UP until stopped or top limit switch is hit
void screw_up()
{
  ros_printf("screw_up()");
  // Safety check
  top_limit_switch_pressed = digitalRead(top_limit_switch_pin) == LOW;
  if (top_limit_switch_pressed)
  {
    ros_printf("Top limit switch pressed. Stopping screw.");
    screw_stop();
    return;
  }
  digitalWrite(dir0, HIGH);
  analogWrite(pwm0, 40);
}

// move screw DOWN until stopped or bottom switch is hit
void screw_down()
{
  ros_printf("screw_down()");
  // Safety check
  bottom_limit_switch_pressed = digitalRead(bottom_limit_switch_pin) == LOW;
  if (bottom_limit_switch_pressed)
  {
    ros_printf("Bottom limit switch pressed. Stopping screw.");
    screw_stop();
    return;
  }
  digitalWrite(dir0, LOW);
  analogWrite(pwm0, 40);
}

// stops the screw
void screw_stop()
{
  ros_printf("screw_stop()");
  analogWrite(pwm0, 0);
}

//-------------------  auger functions  ---------------------
void auger_down()
{
  ros_printf("auger_down()");
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, 100);
}

void auger_up()
{
  ros_printf("auger_up()");
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 100);
}

void auger_stop()
{
  ros_printf("auger_stop()");
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, 0);
}

//-------------------  limit switch  ---------------------
void ISR_bottom()
{
  unsigned long now = millis();
  if (now - last_trigger_time_bottom > debounce_delay)
  {
    last_trigger_time_bottom = now;
    if (digitalRead(bottom_limit_switch_pin) == LOW)
    {
      bottom_limit_switch_pressed = true;
      ros_printf("Bottom limit switch pressed. Stopping screw.");
      screw_stop();
    }
    else
    {
      bottom_limit_switch_pressed = false;
    }
  }
}

void ISR_top()
{
  unsigned long now = millis();
  if (now - last_trigger_time_top > debounce_delay)
  {
    last_trigger_time_top = now;
    if (digitalRead(top_limit_switch_pin) == LOW)
    {
      top_limit_switch_pressed = true;
      ros_printf("Top limit switch pressed. Stopping screw.");
      screw_stop();
    }
    else
    {
      top_limit_switch_pressed = false;
    }
  }
}

void process_serial_aug_command()
{
  static String inputString = "";             // A String to hold incoming data
  static boolean inputStringComplete = false; // Whether the string is complete

  while (SerialUSB.available())
  {
    char inChar = (char)SerialUSB.read(); // Read each character
    if (inChar == '\n')
    {
      inputStringComplete = true; // If newline, input is complete
    }
    else
    {
      inputString += inChar; // Add character to input
    }
  }

  if (inputStringComplete)
  {
    SerialUSB.print("Received: ");
    SerialUSB.println(inputString); // Echo the input for debugging

    // Split the inputString into two integers
    int screwCommand = 0;
    int augerCommand = 0;
    sscanf(inputString.c_str(), "%d %d", &screwCommand, &augerCommand);

    // Process the screw command
    if (screwCommand == 0)
    {
      screw_stop();
    }
    else if (screwCommand == -1)
    {
      screw_down();
    }
    else if (screwCommand == 1)
    {
      screw_up();
    }

    // Process the auger command
    if (augerCommand == 0)
    {
      auger_stop();
    }
    else if (augerCommand == -1)
    {
      auger_down();
    }
    else if (augerCommand == 1)
    {
      auger_up();
    }

    // Clear the string for the next command
    inputString = "";
    inputStringComplete = false;
  }
}

void receive_float_array(float *data, size_t count)
{
  ros_printf("%s", __func__);
  size_t length = count * sizeof(float);
  radio.read(data, length);
}

void radio_loop()
{
  static float transmitter_data[8] = {0};

  ros_printf("%s", __func__);
  bool rx_flag = radio.available();
  // radio.startListening();

  if (rx_flag)
  {
    receive_float_array(transmitter_data, sizeof(transmitter_data) / sizeof(transmitter_data[0]));
    science_data_msg.data_length = sizeof(transmitter_data) / sizeof(transmitter_data[0]);
    science_data_msg.data = transmitter_data;
    science_pub.publish(&science_data_msg);
    nh.spinOnce();
  }
};

//-------------------  Application  ---------------------
void setup()
{
  // DC motor
  pinMode(pwm0, OUTPUT);
  pinMode(dir0, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);

  // stepper
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);

  // NRF24L01
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  // limit switch
  pinMode(bottom_limit_switch_pin, INPUT_PULLUP);
  pinMode(top_limit_switch_pin, INPUT_PULLUP);
  // Attach interrupts for both limit switches
  attachInterrupt(digitalPinToInterrupt(bottom_limit_switch_pin), ISR_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(top_limit_switch_pin), ISR_top, CHANGE);

  nh.initNode();

  nh.subscribe(aug_sub);
  nh.subscribe(stepper_sub);
  nh.advertise(debug_pub);
  nh.advertise(science_pub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  lastTime = millis();
}

void loop()
{
  delay(1);
  nh.spinOnce();
  radio_loop();
}
#endif
