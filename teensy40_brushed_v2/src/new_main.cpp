

// SETUP
//  ros init
//  init encoders, sensors, motors
//  init limit switches + interrupts
// homing?

// LOOP
//  wait for control loop time
//  read encoders
//  read current sensors
//  update PIDs
//  update motors
//  ros publish and spin once

// ros cb

// homing

// ISRs

#include "driver_motor.h"
#include "model_encoder.h"
#include "model_sensor.h"
#include "hardware_pins.h"
#include <Arduino.h>

// #include <ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

// ros::NodeHandle nh;
// float arm_brushed_cmd[3] = {0, 0, 0};
// float arm_brushed_fb[3] = {0, 0, 0};
// void arm_brushed_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
// std_msgs::Float32MultiArray arm_brushed_fb_msg;
// ros::Publisher arm_brushed_fb_pub("/armBrushedFb", &arm_brushed_fb_msg);
// ros::Subscriber<std_msgs::Float32MultiArray> arm_brushed_cmd_sub("/armBrushedCmd", arm_brushed_cmd_cb);

driver_motor wrist_pitch_motor;
// driver_motor wrist_roll_motor;
// driver_motor end_effector_motor;

model_encoder wrist_pitch_encoder;
model_encoder wrist_roll_encoder;
model_encoder end_effector_encoder;

model_sensor wrist_pitch_current;
// model_sensor wrist_roll_current;
// model_sensor end_effector_current;

// void brushed_board_homing();
// void lim1ISR();
// void lim2ISR();
// void lim3ISR();
// void lim4ISR();
// void lim5ISR();
// void lim6ISR();

unsigned long loop_last_time;

void setup()
{
    while (!SerialUSB)
        ;
    SerialUSB.println("on");
    wrist_pitch_encoder.initialize_encoder(1, 190, 43000, 1);
    // wrist_roll_encoder.initialize_encoder(0, 0, 32580, 2);
    // end_effector_encoder.initialize_encoder(0, 0, 43000, 3);

    // cur
    wrist_pitch_current.initialize_sensor(CURRENT_SENSE_A);

    wrist_pitch_motor.initialize_motor(1, 1);
    // wrist_roll_motor.initialize_motor(0, 2);
    // end_effector_motor.initialize_motor(0, 3);

    // pinMode(LIM_1, INPUT);
    // pinMode(LIM_2, INPUT);
    // pinMode(LIM_3, INPUT);
    // pinMode(LIM_4, INPUT);
    // pinMode(LIM_5, INPUT);
    // pinMode(LIM_6, INPUT);
    // attachInterrupt(digitalPinToInterrupt(LIM_1), lim1ISR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(LIM_2), lim2ISR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(LIM_3), lim3ISR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(LIM_4), lim4ISR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(LIM_5), lim5ISR, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(LIM_6), lim6ISR, CHANGE);

    // arm_brushed_fb_msg.data_length = 3;
    // arm_brushed_fb_msg.data = arm_brushed_fb;

    // nh.initNode();
    // nh.advertise(arm_brushed_fb_pub);
    // nh.subscribe(arm_brushed_cmd_sub);
    // nh.negotiateTopics();
    // while (!nh.connected())
    // {
    //     nh.negotiateTopics();
    // }

    // brushed_board_homing();

    loop_last_time = millis();
}

int up = 0;
float setAngle = 190;

void loop()
{
    // delay(CONTROL_LOOP_PERIOD_MS - millis() - loop_last_time);
    while (millis() - loop_last_time < CONTROL_LOOP_PERIOD_MS)
    {
    }
    loop_last_time = millis();
    // SerialUSB.println("here1");
    wrist_pitch_encoder.poll_encoder_angle();
    // wrist_roll_encoder.poll_encoder_angle();
    // end_effector_encoder.poll_encoder_angle();

    if (up)
    {
        setAngle += 0.01;
        if (setAngle >= 195.0)
        {
            up = 0;
        }
    }
    else
    {
        setAngle -= 0.01;
        if (setAngle <= 185.0)
        {
            up = 1;
        }
    }

    wrist_pitch_motor.set_target_angle(setAngle);

    // SerialUSB.println("Before PID");
    wrist_pitch_motor.update_PID(wrist_pitch_encoder.get_angle());
    SerialUSB.print("Set to: ");
    SerialUSB.print(setAngle);
    SerialUSB.print(" Encoder Pitch: ");
    SerialUSB.print(wrist_pitch_encoder.get_angle());
    SerialUSB.print(" Current: ");
    SerialUSB.println(wrist_pitch_current.read_sensor_value());

    // SerialUSB.print(" Encoder Roll: ");
    // SerialUSB.print(wrist_roll_encoder.get_angle());
    // SerialUSB.print(" Encoder EE: ");
    // SerialUSB.println(end_effector_encoder.get_angle());

    // cur

    // wrist_pitch_motor.update_PID(wrist_pitch_encoder.get_angle());
    // wrist_roll_motor.update_PID(wrist_roll_encoder.get_angle());
    // end_effector_motor.update_PID(end_effector_encoder.get_angle());

    // arm_brushed_fb[0] = wrist_pitch_encoder.get_angle();
    // arm_brushed_fb[1] = wrist_roll_encoder.get_angle();
    // arm_brushed_fb[2] = end_effector_encoder.get_angle();

    // arm_brushed_fb_pub.publish(&arm_brushed_fb_msg);
    // nh.spinOnce();
}