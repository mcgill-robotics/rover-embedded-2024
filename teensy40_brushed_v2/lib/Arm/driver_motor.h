/**
 * @file          driver_actuators.h
 * @author        Steve Ding, Oliver Philbin-Briscoe, Maciej Lacki, Colin Gallacher
 * @version       V0.2.0
 * @date          22-February-2022
 * @brief         single actuator controller
 *
 * @attention     For prototype example only
 */

#ifndef DRIVER_JOINT_H
#define DRIVER_JOINT_H

#include "Arduino.h"
#include "hardware_pins.h"

#include "arm_math.h"

#define _MAX_PWM_FREQUENCY 146484.38
#define _PWM_BIT_RESOLUTION 10
#define _PWM_OUTPUT_RESOLUTION 1023

class driver_motor
{
public:
	/**
	 * Initialize a actuator control pins based on port and control parameters
	 *
	 * @param		direction positive actuator rotation direction
	 * @param		port actuator connection port
	 * @param		maxTorque maximum torque limit actuator can output
	 */
	void initialize_motor(uint8_t direction, int port);

	void set_target_angle(float angle);

	int16_t get_output_motor(void);

	float get_target_angle(void);

	void update_PID(float new_angle);

	uint8_t _forward_dir;
	uint8_t _is_circular_joint;
	float _max_limit;
	float _min_limit;

	float _target_angle;
	float _current_angle;

	uint8_t _motor_pwm_pin;
	uint8_t _motor_dir_pin;

	// PID parameters
	arm_pid_instance_f32 pid;
	int16_t _output_motor;

	// TODO: Where are these values from? Are they the aggressive ones or normal ones?
	const float _pi = 3.14159265358979;
	const float _KP = 2.0;
	const float _KI = 1.0;
	const float _KD = 2.0;

	/**
	 * Set PWM signal bit resolution
	 *
	 * @param		resolution new PWM bit resolution
	 */
	void _pwm_set_resolution(uint16_t resolution);

	/**
	 * Hardware PWM pin setup function
	 *
	 * @param		pwmFreq desired hardware PWM frequency
	 */
	void _pwm_setup(float pwmFreq);

	/**
	 * Manually move the motor forwards or backwards
	 *
	 * @param		speed in range -1 to 1
	 */
	void _pwm_write_duty(uint32_t pwmDuty);
};

#endif