#include <Arduino.h>
#include "hardware_pins.h"
#include "driver_motor.h"
#include "model_encoder.h"
#include "model_sensor.h"
#include "motor.h"

#include <ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <memory>

// CONFIGURATION
#define CONFIG_NON_ROS 1
#define CONFIG_DIR_TESTER 2
#define CONFIG_SINGLE_MOTOR_TESTER 3
#define CONFIG_ROS 4
#define BRUSHED_BOARD_CONFIG CONFIG_ROS
#define PID_PERIOD_MS 100

#define OUTA_Pin ENCPIN1_1
#define OUTB_Pin ENCPIN1_2

// ROS
ros::NodeHandle nh;
float arm_brushed_setpoint_ps[3] = {0, 0, 0};
float arm_brushed_angle_ps[3] = {0, 0, 0};
void brushed_board_ros_loop();
void arm_brushed_cmd_cb(const std_msgs::Float32MultiArray &input_msg);
std_msgs::Float32MultiArray arm_brushed_fb_msg;
ros::Publisher arm_brushed_fb_pub("armBrushedFB", &arm_brushed_fb_msg);
ros::Subscriber<std_msgs::Float32MultiArray> arm_brushed_cmd_sub("armBrushedCmd", arm_brushed_cmd_cb);

// DUMB POINTERS
driver_motor mot1;
driver_motor mot2;
driver_motor mot3;

// TESTER VARIABLES
int stage = 0;
int pos = 90;
int tolerance = 0.5;
volatile boolean is_homed = false;

// FUNCTION DECLARATIONS
//void brushed_board_homing();
void brushed_board_ros_loop();

void lim1ISR();
void lim2ISR();
void lim3ISR();
void lim4ISR();
void lim5ISR();
void lim6ISR();

// Define the size of the moving average window
const uint32_t MOVING_AVERAGE_SIZE = 10;
float cur1_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
float cur2_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
float cur3_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
int cur1_voltage_buffer_idx = 0;
int cur2_voltage_buffer_idx = 0;
int cur3_voltage_buffer_idx = 0;

float moving_average(float new_reading, float *buffer, int buffer_size, int *buffer_idx)
{
    float sum = 0;
    float oldest_reading = buffer[*buffer_idx];

    // Add the new reading to the buffer.
    buffer[*buffer_idx] = new_reading;
    *buffer_idx = (*buffer_idx + 1) % buffer_size; // Circular buffer

    // Sum all readings.
    for (int i = 0; i < buffer_size; i++)
    {
        sum += buffer[i];
    }
    sum -= oldest_reading;

    return sum / buffer_size;
}

// Ensure counter is initialized
int counter = 0;

Rotation2d *current_rotation;

// LIMIT SWITCHES
IntervalTimer lim_watchdog_timer;
const unsigned long DEBOUNCE_DELAY_MS = 50;

volatile bool lim1_state = false;
volatile bool lim2_state = false;
volatile bool lim3_state = false;
volatile bool lim4_state = false;
volatile bool lim5_state = false;
volatile bool lim6_state = false;

uint32_t loop_last_time = 0;

void setup()
{
#if BRUSHED_BOARD_CONFIG == CONFIG_ROS
    // Initialize ROS
    nh.initNode();
    nh.advertise(arm_brushed_fb_pub);
    nh.subscribe(arm_brushed_cmd_sub);
    while (!nh.connected())
    {
        nh.negotiateTopics();
        nh.spinOnce();
    }
#endif

    std::unique_ptr<model_encoder> enc1 = std::make_unique<model_encoder>();
    std::unique_ptr<model_encoder> enc2 = std::make_unique<model_encoder>();
    std::unique_ptr<model_encoder> enc3 = std::make_unique<model_encoder>();

    std::unique_ptr<model_sensor> cur1 = std::make_unique<model_sensor>();
    std::unique_ptr<model_sensor> cur2 = std::make_unique<model_sensor>();
    std::unique_ptr<model_sensor> cur3 = std::make_unique<model_sensor>();

    // Initialize encoders
    // 43000 clicks for wrist pitch, 32580 for wrist roll
    // Only using 1 & 2 becase 1 & 3 conflicts
    // Could be 32768 since it's a power of 2
    enc1->initialize_encoder(0, 0, 32580, 1); // new small servo estimate for resolution
    enc2->initialize_encoder(0, 0, 43000, 2);
    // enc3->initialize_encoder(0, 0, 43000, 3);

    // Initialize current sensors
    cur1->initialize_sensor(CURRENT_SENSE_A);
    cur3->initialize_sensor(CURRENT_SENSE_C);
    cur2->initialize_sensor(CURRENT_SENSE_B);

    // Initialize motors
    mot1.attach_encoder(std::move(enc1));
    mot2.attach_encoder(std::move(enc2));
    // mot3.attach_encoder(std::move(enc3));

    mot1.attach_current_sensor(std::move(cur1));
    mot2.attach_current_sensor(std::move(cur2));
    mot3.attach_current_sensor(std::move(cur3));

    // Motor init, forward logic is 1
    mot1.initialize_motor(1, PWMPIN1, DIRPIN1, nSLEEP1, 5.0, 0.0);
    mot2.initialize_motor(1, PWMPIN2, DIRPIN2, nSLEEP2, 5.0, 0.0);
    mot3.initialize_motor(1, PWMPIN3, DIRPIN3, nSLEEP3, 5.0, 0.0);

    // Set motor configuration after initialization
    mot1.set_angle_limit_ps(wrist_pitch_max_angle, wrist_pitch_min_angle);
    mot1.set_gear_ratio(2.0);
    mot1._is_circular_joint = true;

    // Limit Switches
    pinMode(LIM_1, INPUT);
    pinMode(LIM_2, INPUT);
    pinMode(LIM_3, INPUT);
    pinMode(LIM_4, INPUT);
    pinMode(LIM_5, INPUT);
    pinMode(LIM_6, INPUT);
    attachInterrupt(digitalPinToInterrupt(LIM_1), lim1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_2), lim2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_3), lim3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_4), lim4ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_5), lim5ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIM_6), lim6ISR, CHANGE);

    // Watchdog timer to prevent bouncing or stuck limit switches
    lim_watchdog_timer.begin([]()
                             {
        lim1_state = digitalRead(LIM_1);
        lim2_state = digitalRead(LIM_2);
        lim3_state = digitalRead(LIM_3);
        lim4_state = digitalRead(LIM_4);
        lim5_state = digitalRead(LIM_5);
        lim6_state = digitalRead(LIM_6); },
                             DEBOUNCE_DELAY_MS * 1000);

    // Initialize Motor
    // nSLEEP is actually hardwired to 5V so not really necessary
    digitalWrite(nSLEEP1, HIGH);
    digitalWrite(nSLEEP2, HIGH);
    digitalWrite(nSLEEP3, HIGH);

    // Modelling
    current_rotation = Rotation2d::getRotationFromDeg(0);

    //brushed_board_homing();
    // while (!is_homed)
    //     ;
    loop_last_time = micros();
}

void loop()
{
    brushed_board_ros_loop();
}

// WRIST PITCH MAX LIMIT SWITCH
void lim1ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
    {
        is_homed = true;
        mot1._encoder->set_current_angle_es(wrist_pitch_max_angle * mot1._gear_ratio);
        last_interrupt_time = interrupt_time;
        lim1_state = digitalRead(LIM_1);
    }
}

// WRIST PITCH MIN LIMIT SWITCH
void lim2ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
    {
        mot1._encoder->set_current_angle_es(wrist_pitch_min_angle * mot1._gear_ratio);
        last_interrupt_time = interrupt_time;
        lim2_state = digitalRead(LIM_2);
    }
}

void lim3ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
    {
        last_interrupt_time = interrupt_time;
        lim3_state = digitalRead(LIM_3);
    }
}

void lim4ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
    {
        last_interrupt_time = interrupt_time;
        lim4_state = digitalRead(LIM_4);
    }
}

void lim5ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
    {
        last_interrupt_time = interrupt_time;
        lim5_state = digitalRead(LIM_5);
    }
}

void lim6ISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
    {
        last_interrupt_time = interrupt_time;
        lim6_state = digitalRead(LIM_6);
    }
}

// HOMING SEQUENCE, go forward until limit switch is triggered
// void brushed_board_homing()
// {
//     // Only do homing for linear joints
//     if (!mot1._is_circular_joint)
//     {
//         mot1.set_direction(mot1._forward_dir);
//         mot1._pwm_write_duty(40);
//     }
// }
void brushed_board_ros_loop()
{
    // delay(PID_PERIOD_MS);

    uint32_t current_time = micros();
    if (current_time - loop_last_time > 1000000)
    {
        counter = current_time;
        mot1.closed_loop_control_tick();
        mot2.closed_loop_control_tick();
        //mot3.closed_loop_control_tick();
        loop_last_time = current_time;
    }

    // Feedback
    arm_brushed_fb_msg.data_length = 3;
    arm_brushed_angle_ps[0] = mot1.get_current_angle_ps();
    arm_brushed_angle_ps[1] = mot2.get_current_angle_ps();
    //arm_brushed_angle_ps[2] = mot3.get_current_angle_ps();
    arm_brushed_angle_ps[2] = 0.0;
    arm_brushed_fb_msg.data = arm_brushed_angle_ps;
    arm_brushed_fb_pub.publish(&arm_brushed_fb_msg);

    nh.spinOnce();
    delay(1);
}

void arm_brushed_cmd_cb(const std_msgs::Float32MultiArray &input_msg)
{
    arm_brushed_setpoint_ps[0] = input_msg.data[0];
    arm_brushed_setpoint_ps[1] = input_msg.data[1];
    arm_brushed_setpoint_ps[2] = input_msg.data[2];
    mot1.set_target_angle_ps(arm_brushed_setpoint_ps[0]);
    mot2.set_target_angle_ps(arm_brushed_setpoint_ps[1]);
    mot3.set_target_angle_ps(arm_brushed_setpoint_ps[2]);
}