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

#include "model_encoder.h"
#include "model_sensor.h"

#include "pid.h"

#include <memory>

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
	void initialize_motor(uint8_t directionMotor, uint8_t motorPWMPin, uint8_t motorDirPin, uint8_t motorFaultPin, float maxTorqueMotor, float torqueConstantMotor);

	/**
	 * Set specific torque target torque
	 *
	 * @param		torque torque to be output
	 */
	void set_target_torque(float torque);

	void set_target_position(float position);

	void set_target_speed(float speed);

	void set_direction(uint8_t direction);

	void set_gear_ratio(float gear_ratio);

	void set_is_circular_joint(boolean is_circular_joint);
	// Set the logic that defines the forward direction of the motor
	void set_forward_dir(uint8_t forward_dir);

	/// @brief Sets the positive direction of the motor
	/// @param direction Positive direction of motor (CW or CCW)
	void setDirection(uint8_t direction);

	/// @brief Sets the control period of the motor PID loop
	/// @param period
	void set_control_period(float period);

	float get_target_torque(void);

	float get_output_motor(void);

	float get_target_position(void);

	float get_target_speed(void);
	/**
	 * Control torque output
	 *
	 * @param		mtrCurrent detected motor current at time index
	 * @param		powerState main power state
	 */
	void torque_control(float motorCur);

	void position_control(float motorPos);

	void speed_control(float motorSpeed);

	/**
	 * Set hardware PWM duty cycle
	 *
	 * @param		pwmPin hardware PWM pin to output signal
	 * @param		pwmDuty bit resolution value of desired PWM duty cycle
	 */
	void _pwm_write_duty(uint8_t pwmPin, uint32_t pwmDuty);

	void attach_encoder(std::unique_ptr<model_encoder> encoder)
	{
		_encoder = std::move(encoder);
	}

	void attach_current_sensor(std::unique_ptr<model_sensor> current_sensor)
	{
		_current_sensor = std::move(current_sensor);
	}

	void closed_loop_control_tick();

	// private:
	// model_encoder *_encoder = nullptr;
	// model_sensor *_current_sensor = nullptr;
	std::unique_ptr<model_encoder> _encoder;
	std::unique_ptr<model_sensor> _current_sensor;

	// JOINT CONFIG.
	boolean _is_circular_joint;
	uint8_t _forward_dir;


	uint8_t _motor_pwm_pin;
	uint8_t _motor_dir_pin;
	uint8_t _motor_fault_pin;

	uint8_t _rotational_direction_motor;
	float _motor_max_torque;
	float _motor_max_speed;
	float _motor_max_position;
	float _motor_min_position;
	float _current_angle_es;
	float _angle_full_turn;

	float _target_torque;
	float _target_position;
	float _targetSpeed;

	// PID parameters
	float _output_motor;
	float _ctrl_period;
	float _sampling_period;

	PID *pid_instance;

	float _torque_constant_motor;
	float _gear_ratio;

	// TODO: Where are these values from? Are they the aggressive ones or normal ones?
	const float _pi = 3.14159265358979;
	const float _KP = 4.0;
	const float _KI = 2.0;
	const float _KD = 1.0;

	// Filter params
	//  kalman filter parameters

	const float _ERR_ESTIMATE_INIT = 1.877e-3;
	const float _LAST_ESTIMATE_INIT = 0;
	const float _ERR_MEASURE_INIT = 1.877e-3;
	const float _Q_INIT = 0.0044;

	float _err_estimate;
	float _last_estimate;
	float _err_measure;
	float _q;

	float _kalman_gain;
	float _current_estimate;

	// Error parameters and variables
	const float _maxError = 2.0; // 10000.0; //2.0;

	float _error;
	float _previous_error;

	float _error_int;
	float _error_dir;

	// const float _MAX_PWM_FREQUENCY = 146484.38;
	// const uint8_t _PWM_BIT_RESOLUTION = 10;
	// const uint32_t _PWM_OUTPUT_RESOLUTION = 1023;

	/**
	 * Set PWM signal bit resolution
	 *
	 * @param		resolution new PWM bit resolution
	 */
	void _pwm_set_resolution(uint16_t resolution);

	/**
	 * Hardware PWM pin setup function
	 *
	 * @param		pwmPin hardware PWM pin to setup
	 * @param		pwmFreq desired hardware PWM frequency
	 */
	void _pwm_setup(uint8_t pwmPin, float pwmFreq);
};

#endif