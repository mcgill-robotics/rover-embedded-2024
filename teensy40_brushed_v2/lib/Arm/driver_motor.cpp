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
void driver_motor::initialize_motor(uint8_t direction_motor, uint8_t motor_pwn_pin, uint8_t motor_dir_pin, uint8_t motor_fault_pin, float max_torque_motor, float torque_constant_motor)
{
	_rotational_direction_motor = direction_motor;
	_motor_max_torque = max_torque_motor;
	_torque_constant_motor = torque_constant_motor;
	_target_torque = 0.0;
	_target_position = 0.0;
	_output_motor = 0;

	_ctrl_period = 0.0;
	_sampling_period = 0.0;

	_motor_dir_pin = motor_dir_pin;
	_motor_fault_pin = motor_fault_pin;
	_motor_pwm_pin = motor_pwn_pin;

	// kalman params
	_kalman_gain = 0;
	_current_estimate = 0;

	_err_estimate = _ERR_ESTIMATE_INIT;
	_last_estimate = _LAST_ESTIMATE_INIT;
	_err_measure = _ERR_MEASURE_INIT;
	_q = _Q_INIT;

	_error = 0.0;
	_previous_error = 0.0;

	_error_int = 0.0;
	_error_dir = 0.0;

	// initializes pins and resolution
	// _pwm_set_resolution(_PWM_BIT_RESOLUTION);

	pinMode(_motor_pwm_pin, OUTPUT);
	pinMode(_motor_dir_pin, OUTPUT);
	pinMode(_motor_fault_pin, OUTPUT);

	// _pwm_setup(_motor_pwm_pin, _MAX_PWM_FREQUENCY); // Sets frequency of pwm
}

/// @brief Sets the torque and writes the PWM value to the motor
/// @param torque Torque value to be written to the motor
void driver_motor::set_target_torque(float torque)
{
	_target_torque = torque;
}

float driver_motor::get_target_torque(void)
{
	return _target_torque;
}

float driver_motor::get_output_motor(void)
{
	return _output_motor;
}

void driver_motor::set_target_position(float position){
	_target_position = position;
}

float driver_motor::get_target_position(void)
{
	return _target_position;
}

void driver_motor::set_control_period(float period)
{
	_ctrl_period = period;
	_sampling_period = _ctrl_period * 1e-6;
}

void driver_motor::closed_loop_control_tick()
{
	//TODO: PID implementation 

}

void driver_motor::torque_control(float motor_cur)
{

	bool direction = LOW;
	float motor_current;

	if (_rotational_direction_motor)
	{
		motor_current = motor_cur;
	}
	else
	{
		motor_current = -1 * motor_cur;
	}

	_kalman_gain = _err_estimate / (_err_estimate + _err_measure);
	_current_estimate = _last_estimate + _kalman_gain * (motor_current - _last_estimate);

	_err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
	_last_estimate = _current_estimate;

	_error = _target_torque - _current_estimate * _torque_constant_motor;
	_error_int += _error * _sampling_period;

	// Bound the integral error such that it cannot saturate the motors
	// The typical error is less than 1
	if (_error_int > _maxError)
	{
		_error_int = _maxError;
	}
	else if (_error_int < -_maxError)
	{
		_error_int = -_maxError;
	}

	// Calculate output
	_output_motor = _error * _KP + _error_int * _KI + _error_dir * _KD;

	if (_output_motor == _output_motor)		//TODO: why are we comparing it to itself?
	{
		_output_motor *= _PWM_OUTPUT_RESOLUTION / _motor_max_torque;
	}
	else
	{
		_output_motor = 0.0;

		_error = 0.0;
		_previous_error = 0.0;

		_error_int = 0.0;
		_error_dir = 0.0;
	}

	// Set direction
	if (_rotational_direction_motor == 0)
	{
		direction = !direction;
	}

	if (_output_motor <= 0)
	{
		digitalWrite(_motor_dir_pin, direction);
	}
	else
	{
		digitalWrite(_motor_dir_pin, !direction);
	}

	// output torque
	_output_motor = abs(_output_motor);
	if (_output_motor > _PWM_OUTPUT_RESOLUTION)
	{
		_pwm_write_duty(_motor_pwm_pin, _PWM_OUTPUT_RESOLUTION);
	}
	else
	{
		_pwm_write_duty(_motor_pwm_pin, (uint32_t)_output_motor);
	}
}
/// @brief Sets the positive direction of the motor
/// @param direction new positive direction of the motor
void driver_motor::setDirection(uint8_t direction)
{
	_rotational_direction_motor = direction;
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
void driver_motor::_pwm_setup(uint8_t pwmPin, float pwmFreq)
{
	analogWriteFrequency(pwmPin, pwmFreq);
}

/// @brief Writes the PWM value to the motor
/// @param pwmPin Pin used for PWM modulation to the motor
/// @param pwmDuty Duty cyle value to be written to PWM
void driver_motor::_pwm_write_duty(uint8_t pwmPin, uint32_t pwmDuty)
{
	analogWrite(pwmPin, pwmDuty);
}
