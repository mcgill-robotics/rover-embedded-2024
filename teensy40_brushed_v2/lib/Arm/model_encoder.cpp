/**
 * @file          model_encoder.cpp
 * @author        Vincent Boucher, Steve Ding
 * @version       V1.0.0
 * @date          30-May-2023
 * @brief         encoder object
 *
 * @attention     For prototype example only
 *				  V1.0.0 - created
 *
 */
#include "model_encoder.h"

void model_encoder::initialize_encoder(uint8_t rotationalDirection, float offset, float resolution, uint8_t port)
{

    _offset = offset;
    _resolution = resolution;
    _port = port;

    _angularVelocity = 0.0;
    _position = _offset * (_resolution / 360.0);

    switch (port)
    {
    case 1:
        _pinA = ENCPIN1_1;
        _pinB = ENCPIN1_2;
        break;
    case 2:
        _pinA = ENCPIN2_1;
        _pinB = ENCPIN2_2;
        break;
    case 3:
        _pinA = ENCPIN3_1;
        _pinB = ENCPIN3_2;
        break;
    default:
        _pinA = 0;
        _pinB = 0;
        break;
    }

    if (rotationalDirection)
    {
        uint8_t tempPin = _pinA;
        _pinA = _pinB;
        _pinB = tempPin;
    }

    _encoder = new QuadEncoder(port, _pinA, _pinB);

    _encoder->setInitConfig();
    _encoder->EncConfig.positionInitialValue = _position;
    _encoder->init();

    _velocityEstimation.initialize_parameters(_resolution, 8.0, 1000000.0, 1, 200000);
}

void model_encoder::reset_encoder()
{
    _position = _offset * (_resolution / 360.0);
    _encoder->write(_position);
    _encoder->init();
}

void model_encoder::position_reset_encoder(float offset)
{
    _position = offset * (_resolution / 360.0);
    _encoder->write(_position);
}

float model_encoder::read_encoder_angle()
{
    _position = _encoder->read();
    _angle = (_position >= 0) ? (_position % _resolution) : _resolution - (abs(_position) % _resolution);
    _angle = (360.0 / _resolution) * _angle;
    _velocityEstimation.update_readings(_position * (360.0 / _resolution), micros());
    return _angle;
}

float model_encoder::getAngle()
{
    return _angle;
}

void model_encoder::set_parameters(uint8_t direction, float offset, float resolution)
{
    if (direction)
    {
        uint8_t tempPin = _pinA;
        _pinA = _pinB;
        _pinB = tempPin;
    }
    _encoder->enc_xbara_mapping(_pinA, PHASEA, 0);
    _encoder->enc_xbara_mapping(_pinB, PHASEB, 0);
    _encoder->disableInterrupts(_positionROEnable);
    _encoder->disableInterrupts(_positionRUEnable);

    _offset = offset;
    _resolution = resolution;
    _position = _offset * (_resolution / 360.0);

    _encoder->setInitConfig();
    _encoder->EncConfig.positionInitialValue = _position;
    _encoder->init();
}

void model_encoder::velocityEstimation(void)
{
    _angularVelocity = _velocityEstimation.foaw(0);
}

float model_encoder::getVelocity(void)
{
    return _angularVelocity;
}