/**
 * @file          model_sensor.cpp
 * @author        Vincent Boucher, Steve Ding
 * @version       V1.0.0
 * @date          30-May-2023
 * @brief         current sensor object
 *
 * @attention     For prototype example only
 *				  V1.0.0 - created
 *					
 */
#include "model_sensor.h"

void model_sensor::initialize_sensor(uint8_t pin) {
	_sensorValue = 0;
	_sensorOffset = 0;

	_pin = pin;

	_initialize_adc();

	pinMode(pin, INPUT);

	delay(10);

    for (uint8_t i = 0; i < 40; i++) {
        _update_sensor_value();
        _sensorOffset = _sensorOffset + _sensorValue;
    }

    _sensorOffset = _sensorOffset / 40;

    _sensorOffset = 1862 - _sensorOffset;
}


void model_sensor::reset_sensor(void) {
	_sensorValue = 0;
}


float model_sensor::read_sensor_value(void) {
    _update_sensor_value();
	float sensorValue = _sensorValue + _sensorOffset;
	_current16 = sensorValue - 1862;
    sensorValue = sensorValue * (3.3/4095.0);
	_current = (sensorValue - 1.5) * 2;
    return _current;
}

float model_sensor::getCurrent(void){
	return _current;
}

int16_t model_sensor::getCurrent16(void){
	return _current16;
}


void model_sensor::_update_sensor_value(void) {
    // if (_pin == CURRENT_SENSE_A1 ||
	// 		_pin == CURRENT_SENSE_B1 ||
	// 		_pin == CURRENT_SENSE_A2 ||
	// 		_pin == CURRENT_SENSE_B2) {
	//     _sensorValue = _adc->adc1->analogRead(_pin);
    // }else{
        _sensorValue = _adc->adc0->analogRead(_pin);
    // }
}



void model_sensor::_initialize_adc(void) {
	
	_adc = new ADC();
	
	_adc->adc0->setAveraging(4);
	_adc->adc0->setResolution(12);
	_adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
	_adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

	_adc->adc1->setAveraging(4);
	_adc->adc1->setResolution(12);
	_adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
	_adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
}