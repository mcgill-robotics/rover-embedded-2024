/**
 * @file          driver_motor.cpp
 * @author        Vincent Boucher, Steve Ding
 * @version       V1.0.0
 * @date          30-May-2023
 * @brief         motor object
 *
 * @attention     For prototype example only
 *				  V1.0.0 - created
 *
 */
#include "driver_motor.h"

/// @brief Initializes the motor object
/// @param direction Positive direction for motor
/// @param port Port used for motor connection
/// @param maxTorque Maximum torque motor can output
void driver_motor::initialize_motor(uint8_t direction, int port)
{

	switch (port)
	{

	case 1: // motor 1
		_is_circular_joint = MOTOR1_CONTINOUS;
		_max_limit = MOTOR1_LIMIT_MAX;
		_min_limit = MOTOR1_LIMIT_MIN;
		_motor_pwm_pin = PWMPIN1;
		_motor_dir_pin = DIRPIN1;
		break;
	case 2: // motor 2
		_is_circular_joint = MOTOR2_CONTINOUS;
		_max_limit = MOTOR2_LIMIT_MAX;
		_min_limit = MOTOR2_LIMIT_MIN;
		_motor_pwm_pin = PWMPIN2;
		_motor_dir_pin = DIRPIN2;
		break;
	case 3: // motor 3
		_is_circular_joint = MOTOR3_CONTINOUS;
		_max_limit = MOTOR3_LIMIT_MAX;
		_min_limit = MOTOR3_LIMIT_MIN;
		_motor_pwm_pin = PWMPIN3;
		_motor_dir_pin = DIRPIN3;
		break;
	default:
		break;
	}

	_target_angle = 0.0;
	_output_motor = 0;
	_forward_dir = direction;

	// initializes pins and resolution
	_pwm_set_resolution(_PWM_BIT_RESOLUTION);

	pinMode(_motor_pwm_pin, OUTPUT);
	pinMode(_motor_dir_pin, OUTPUT);

	_pwm_setup(_MAX_PWM_FREQUENCY); // Sets frequency of pwm

	// 24V motor therefore max is 24 and min is -24
	pid.Kp = _KP; // Proportional gain
	pid.Ki = _KI; // Integral gain
	pid.Kd = _KD; // Derivative gain

	arm_pid_init_f32(&pid, 1);
}

int16_t driver_motor::get_output_motor(void)
{
	return _output_motor;
}

void driver_motor::set_target_angle(float angle)
{
	// float new_angle = (_forward_dir) ? angle : (360.0 - angle);

	if (!_is_circular_joint)
	{
		if (angle < _min_limit)
		{
			_target_angle = _min_limit;
		}
		else if (angle > _max_limit)
		{
			_target_angle = _max_limit;
		}
		else
		{
			_target_angle = angle;
		}
	}
	else
	{
		_target_angle = angle;
	}
	return;
}

float driver_motor::get_target_angle(void)
{
	return _target_angle;
}

/// @brief Sets the resolution of the motor
/// @param resolution new resolution of the motor
void driver_motor::_pwm_set_resolution(uint16_t resolution)
{

	analogWriteResolution(resolution);
}

/// @brief Sets the PWM frequency of the motor
/// @param pwmPin Pin used for PWM modulation to the motor
/// @param pwmFreq Frequency to be set for PWM
void driver_motor::_pwm_setup(float pwmFreq)
{
	analogWriteFrequency(_motor_pwm_pin, pwmFreq);
}

/// @brief Writes the PWM value to the motor
/// @param pwmPin Pin used for PWM modulation to the motor
/// @param pwmDuty Duty cyle value to be written to PWM
void driver_motor::_pwm_write_duty(uint32_t pwmDuty)
{
	analogWrite(_motor_pwm_pin, pwmDuty);
}

void driver_motor::update_PID(float new_angle)
{
	_current_angle = new_angle;
	float error = _current_angle - _target_angle;
	// SerialUSB.println(_target_angle);

	_output_motor = constrain((int16_t)arm_pid_f32(&pid, error), -1024, 1024);
	// _output_motor = constrain(0, -1024, 1024);
	if (_output_motor >= 0.0)
	{
		digitalWrite(_motor_dir_pin, _forward_dir);
	}
	else
	{
		digitalWrite(_motor_dir_pin, (_forward_dir) ? 0 : 1);
	}
	_pwm_write_duty((uint32_t)abs(_output_motor));
}