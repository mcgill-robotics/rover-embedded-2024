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
#include <Arduino.h>
#include "model_encoder.h"

void model_encoder::initialize_encoder(uint8_t rotationalDirection, float offset, float resolution, uint8_t port)
{
    _offset = offset;
    _resolution = resolution;
    _port = port;

    _angle = offset;
    _quad_enc_pos = _offset * (_resolution / 360.0);
    _last_angle = 0.0;

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
    _encoder->EncConfig.positionInitialValue = _quad_enc_pos;
    _encoder->EncConfig.revolutionCountCondition = 1;
    _encoder->init();
}

void model_encoder::reset_encoder()
{
    _quad_enc_pos = _offset * (_resolution / 360.0);
    _encoder->write(_quad_enc_pos);
    _encoder->init();
}

void model_encoder::poll_encoder_angle()
{
    _quad_enc_pos = _encoder->read();

    _angle = (_quad_enc_pos >= 0) ? (_quad_enc_pos % _resolution) : _resolution - (abs(_quad_enc_pos) % _resolution);
    _angle = (360.0 / _resolution) * _angle;
}

float model_encoder::get_angle()
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
    _quad_enc_pos = _offset * (_resolution / 360.0);

    _encoder->setInitConfig();
    _encoder->EncConfig.positionInitialValue = _quad_enc_pos;
    _encoder->init();
}