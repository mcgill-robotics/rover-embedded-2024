#include "common.h"

#if USE_ROS_FIRMWARE == 2
#include <Arduino.h>
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>

ros::NodeHandle nh;
void stepper_cb(const std_msgs::Float64MultiArray &input_msg);
ros::Subscriber<std_msgs::Float64MultiArray> stepper_sub("/stepperCmd", stepper_cb);
void aug_cb(const std_msgs::Float64MultiArray &input_msg);
ros::Subscriber<std_msgs::Float64MultiArray> aug_sub("/augerCmd", aug_cb);

void screw_up();
void screw_down();
void screw_stop();
void auger_up();
void auger_down();
void auger_stop();

const int stepsPerRevolution = 800; // 1/4 step
volatile float degreesPerStep = 360.0 / stepsPerRevolution;
volatile float degreesToTurn = 0.0;
volatile int currentStepperStepPinState = 0;
volatile int STEPPER_STEP_PIN = 1;
volatile int STEPPER_DIR_PIN = 0;
volatile int STEPPER_DELAY = 7000;
const int STEPPER_M0_PIN = 6;
const int STEPPER_M1_PIN = 7;
const int STEPPER_M2_PIN = 8;

// screw
const int pwm0 = 2;
const int dir0 = 3;

// auger
const int pwm1 = 4;
const int dir1 = 5;

#define debounce_delay 100
const int top_limit_switch_pin = 17;    // top limit switch
const int bottom_limit_switch_pin = 18; // bottom limit switch
volatile unsigned long last_trigger_time_top = 0;
volatile unsigned long last_trigger_time_bottom = 0;
volatile bool top_limit_switch_pressed = false;    // true if at very top
volatile bool bottom_limit_switch_pressed = false; // true if at very bottom

IntervalTimer stepperTimer;

void ISR_top();
void ISR_bottom();
void ISR_stepper();

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

// Every publish to /science will rotate the stepper motor by the amount in the message
void stepper_cb(const std_msgs::Float64MultiArray &input_msg)
{
    degreesToTurn += input_msg.data[0];
}

//-------------------  DC motor functions  ---------------------
// move screw UP until stopped or top limit switch is hit
void screw_up()
{
    // Safety check
    top_limit_switch_pressed = digitalRead(top_limit_switch_pin) == LOW;
    if (top_limit_switch_pressed)
    {
        screw_stop();
        return;
    }
    digitalWrite(dir0, HIGH);
    analogWrite(pwm0, 150);
}

// move screw DOWN until stopped or bottom switch is hit
void screw_down()
{
    // Safety check
    bottom_limit_switch_pressed = digitalRead(bottom_limit_switch_pin) == LOW;
    if (bottom_limit_switch_pressed)
    {
        screw_stop();
        return;
    }
    digitalWrite(dir0, LOW);
    analogWrite(pwm0, 150);
}

// stops the screw
void screw_stop()
{
    analogWrite(pwm0, 0);
}

//-------------------  auger functions  ---------------------
void auger_down()
{
    digitalWrite(dir1, LOW);
    analogWrite(pwm1, 150);
}

void auger_up()
{
    digitalWrite(dir1, HIGH);
    analogWrite(pwm1, 150);
}

void auger_stop()
{
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
            screw_stop();
        }
        else
        {
            top_limit_switch_pressed = false;
        }
    }
}

//-------------------  Application  ---------------------
void setup()
{
    // DC motor
    pinMode(pwm0, OUTPUT);
    pinMode(dir0, OUTPUT);
    pinMode(pwm1, OUTPUT);
    pinMode(dir1, OUTPUT);

    pinMode(STEPPER_M0_PIN, OUTPUT);
    pinMode(STEPPER_M1_PIN, OUTPUT);
    pinMode(STEPPER_M2_PIN, OUTPUT);
    digitalWrite(STEPPER_M0_PIN, LOW); // 1/4 step 0 1 0
    digitalWrite(STEPPER_M1_PIN, HIGH);
    digitalWrite(STEPPER_M2_PIN, LOW);

    // stepper
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);

    // limit switch
    pinMode(bottom_limit_switch_pin, INPUT_PULLUP);
    pinMode(top_limit_switch_pin, INPUT_PULLUP);
    // Attach interrupts for both limit switches
    attachInterrupt(digitalPinToInterrupt(bottom_limit_switch_pin), ISR_bottom, CHANGE);
    attachInterrupt(digitalPinToInterrupt(top_limit_switch_pin), ISR_top, CHANGE);
    stepperTimer.begin(ISR_stepper, STEPPER_DELAY);

    nh.initNode();

    nh.subscribe(aug_sub);
    nh.subscribe(stepper_sub);

    nh.negotiateTopics();
    while (!nh.connected())
    {
        nh.negotiateTopics();
    }
}

int stepper_dir = 0;

void loop()
{
    delay(1);
    nh.spinOnce();
}

void ISR_stepper()
{

    if (currentStepperStepPinState)
    {
        digitalWrite(STEPPER_STEP_PIN, LOW);
        currentStepperStepPinState = 0;
        return;
    }

    if (abs(degreesToTurn) >= degreesPerStep)
    {
        (degreesToTurn < 0) ? digitalWrite(STEPPER_DIR_PIN, LOW) : digitalWrite(STEPPER_DIR_PIN, HIGH); // change dir pin
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        degreesToTurn -= (degreesToTurn < 0) ? degreesPerStep * -1 : degreesPerStep;
    }
}
#endif