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


#define _MAX_PWM_FREQUENCY 146484.38
#define _PWM_BIT_RESOLUTION 10
#define _PWM_OUTPUT_RESOLUTION 1023

class driver_motor {
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

	/// @brief Sets the positive direction of the motor
	/// @param direction Positive direction of motor (CW or CCW)
	void setDirection(uint8_t direction);

	/// @brief Sets the control period of the motor PID loop
	/// @param period 
	void set_control_period(float period);

	float getSetTorque(void);

	float getOutputMotor(void);

	float getSetPosition(void);

	float getSetSpeed(void);
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

private:
	uint8_t _motorPwmPin;
	uint8_t _motorDirPin;
	uint8_t _motorFaultPin;

	uint8_t _rotationalDirectionMotor;
	float _motorMaxTorque;
	float _motorMaxSpeed;
	float _motorMaxPosition;
	float _motorMinPosition;

	float _targetTorque;
	float _targetPosition;
	float _targetSpeed;

	//PID parameters
	float _outputMotor;
	float _ctrlPeriod;
	float _samplingPeriod;

	float _torqueConstantMotor;

	const float _pi = 3.14159265358979;
	const float _KP = 4.0;
	const float _KI = 2.0;
	const float _KD = 1.0;

	//Filter params
	// kalman filter parameters
	
	const float _ERR_ESTIMATE_INIT = 1.877e-3;
	const float _LAST_ESTIMATE_INIT = 0;
	const float _ERR_MEASURE_INIT = 1.877e-3;
	const float _Q_INIT = 0.0044;
	

	float _errEstimate;
	float _lastEstimate;
	float _errMeasure;
	float _q;


	float _kalmanGain;
	float _currentEstimate;

	// Error parameters and variables
	const float _maxError = 2.0; // 10000.0; //2.0; 
	
	float _error;
	float _previousError;

	float _errorInt;
	float _errorDir;


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