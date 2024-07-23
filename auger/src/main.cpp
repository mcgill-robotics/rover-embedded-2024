#include "common.h"

#if USE_ROS_FIRMWARE == 0
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
  Serial.printf("screw_up()");
  // Safety check
  top_limit_switch_pressed = digitalRead(top_limit_switch_pin) == LOW;
  if (top_limit_switch_pressed)
  {
    Serial.printf("Top limit switch pressed. Stopping screw.");
    screw_stop();
    return;
  }
  digitalWrite(dir0, HIGH);
  analogWrite(pwm0, 40);
}

// move screw DOWN until stopped or bottom switch is hit
void screw_down()
{
  Serial.printf("screw_down()");
  // Safety check
  bottom_limit_switch_pressed = digitalRead(bottom_limit_switch_pin) == LOW;
  if (bottom_limit_switch_pressed)
  {
    Serial.printf("Bottom limit switch pressed. Stopping screw.");
    screw_stop();
    return;
  }
  digitalWrite(dir0, LOW);
  analogWrite(pwm0, 40);
}

// stops the screw
void screw_stop()
{
  Serial.printf("screw_stop()");
  analogWrite(pwm0, 0);
}

//-------------------  auger functions  ---------------------
void auger_down()
{
  Serial.printf("auger_down()");
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, 100);
}

void auger_up()
{
  Serial.printf("auger_up()");
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 100);
}

void auger_stop()
{
  Serial.printf("auger_stop()");
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
      Serial.printf("Bottom limit switch pressed. Stopping screw.");
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
      Serial.printf("Top limit switch pressed. Stopping screw.");
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

void receiveFloatArray(float *data, size_t count)
{
  Serial.printf("%s()\n", __func__);
  size_t length = count * sizeof(float);
  radio.read(data, length);
  for (uint32_t i = 0; i < length; i++)
  {
    // Print byte in XX:XX format
    Serial.printf("%02X", ((uint8_t *)data)[i] & 0xFF);
    Serial.print(":");
  }
}

void radio_loop()
{
  bool rx_flag = radio.available();
  // radio.startListening();

  if (rx_flag)
  {
    Serial.printf("%s() rx_flag\r\n", __func__);
    float transmitter_data[8];
    receiveFloatArray(transmitter_data, sizeof(transmitter_data) / sizeof(transmitter_data[0]));
    Serial.printf("pH data: %f, %f, %f, %f\r\n", transmitter_data[0], transmitter_data[1], transmitter_data[2], transmitter_data[3]);
    Serial.printf("moisture data: %f, %f, %f, %f\r\n", transmitter_data[4], transmitter_data[5], transmitter_data[6], transmitter_data[7]);
  }
};

//-------------------  Application  ---------------------
void setup()
{
  SerialUSB.begin(115200);

  // DC motor
  pinMode(pwm0, OUTPUT);
  pinMode(dir0, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);

  // stepper
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);

  // nrf
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();

  // limit switch
  pinMode(bottom_limit_switch_pin, INPUT_PULLUP);
  pinMode(top_limit_switch_pin, INPUT_PULLUP);
  // Attach interrupts for both limit switches
  attachInterrupt(digitalPinToInterrupt(bottom_limit_switch_pin), ISR_bottom, CHANGE);
  attachInterrupt(digitalPinToInterrupt(top_limit_switch_pin), ISR_top, CHANGE);

  lastTime = millis();
}

void loop()
{
  delay(1);
  if (millis() - lastTime > 1000)
  {
    Serial.printf("1 second has passed\r\n");
    lastTime = millis();
  }
  process_serial_aug_command();
  radio_loop();
  // if (radio.available())
  // {
  //   char text[32] = "";
  //   radio.read(&text, sizeof(text));
  //   Serial.println(text);
  // }
}
#endif