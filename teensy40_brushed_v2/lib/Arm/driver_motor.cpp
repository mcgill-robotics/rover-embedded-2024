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
void driver_motor::initialize_motor(uint8_t directionMotor, uint8_t motorPWMPin, uint8_t motorDirPin, uint8_t motorFaultPin, float maxTorqueMotor, float torqueConstantMotor)
{
	_rotationalDirectionMotor = directionMotor;
	_motorMaxTorque = maxTorqueMotor;
	_targetTorque = 0.0;
	_outputMotor = 0;

	_ctrlPeriod = 0.0;
	_samplingPeriod = 0.0;

	_motorDirPin = motorDirPin;
	_motorFaultPin = motorFaultPin;
	_motorPwmPin = motorPWMPin;

	// kalman params
	_kalmanGain = 0;
	_currentEstimate = 0;

	_errEstimate = _ERR_ESTIMATE_INIT;
	_lastEstimate = _LAST_ESTIMATE_INIT;
	_errMeasure = _ERR_MEASURE_INIT;
	_q = _Q_INIT;

	_error = 0.0;
	_previousError = 0.0;

	_errorInt = 0.0;
	_errorDir = 0.0;

	// initializes pins and resolution
	_pwm_set_resolution(_PWM_BIT_RESOLUTION);

	pinMode(_motorPwmPin, OUTPUT);
	pinMode(_motorDirPin, OUTPUT);
	pinMode(_motorFaultPin, OUTPUT);

	_pwm_setup(_motorPwmPin, _MAX_PWM_FREQUENCY); // Sets frequency of pwm
}

/// @brief Sets the torque and writes the PWM value to the motor
/// @param torque Torque value to be written to the motor
void driver_motor::set_target_torque(float torque)
{
	_targetTorque = torque;
}

float driver_motor::getSetTorque(void)
{
	return _targetTorque;
}

float driver_motor::getOutputMotor(void)
{
	return _outputMotor;
}

void driver_motor::set_control_period(float period)
{
	_ctrlPeriod = period;
	_samplingPeriod = _ctrlPeriod * 1e-6;
}

void driver_motor::closed_loop_control_tick()
{
	
}

void driver_motor::torque_control(float motorCur)
{

	bool direction = LOW;
	float motorCurrent;

	if (_rotationalDirectionMotor)
	{
		motorCurrent = motorCur;
	}
	else
	{
		motorCurrent = -1 * motorCur;
	}

	_kalmanGain = _errEstimate / (_errEstimate + _errMeasure);
	_currentEstimate = _lastEstimate + _kalmanGain * (motorCurrent - _lastEstimate);

	_errEstimate = (1.0 - _kalmanGain) * _errEstimate + fabs(_lastEstimate - _currentEstimate) * _q;
	_lastEstimate = _currentEstimate;

	_error = _targetTorque - _currentEstimate * _torqueConstantMotor;
	_errorInt += _error * _samplingPeriod;

	// Bound the integral error such that it cannot saturate the motors
	// The typical error is less than 1
	if (_errorInt > _maxError)
	{
		_errorInt = _maxError;
	}
	else if (_errorInt < -_maxError)
	{
		_errorInt = -_maxError;
	}

	// Calculate output
	_outputMotor = _error * _KP + _errorInt * _KI + _errorDir * _KD;

	if (_outputMotor == _outputMotor)
	{
		_outputMotor *= _PWM_OUTPUT_RESOLUTION / _motorMaxTorque;
	}
	else
	{
		_outputMotor = 0.0;

		_error = 0.0;
		_previousError = 0.0;

		_errorInt = 0.0;
		_errorDir = 0.0;
	}

	// Set direction
	if (_rotationalDirectionMotor == 0)
	{
		direction = !direction;
	}

	if (_outputMotor <= 0)
	{
		digitalWrite(_motorDirPin, direction);
	}
	else
	{
		digitalWrite(_motorDirPin, !direction);
	}

	// output torque
	_outputMotor = abs(_outputMotor);
	if (_outputMotor > _PWM_OUTPUT_RESOLUTION)
	{
		_pwm_write_duty(_motorPwmPin, _PWM_OUTPUT_RESOLUTION);
	}
	else
	{
		_pwm_write_duty(_motorPwmPin, (uint32_t)_outputMotor);
	}
}
/// @brief Sets the positive direction of the motor
/// @param direction new positive direction of the motor
void driver_motor::setDirection(uint8_t direction)
{
	_rotationalDirectionMotor = direction;
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
