/**
 * @file          model_sensor_sensors.h
 * @author        Steve Ding, Oliver Philbin-Briscoe, Colin Gallacher
 * @version       V1.1.0
 * @date          26-Janurary-2023
 * @brief         generic sensor object
 *
 * @attention     For prototype example only
 *				  V1.1.0 - added offset calculations for current and inkwell sensors
 *
 */

#ifndef MODEL_ENCODER_H
#define MODEL_ENCODER_H

#include "hardware_pins.h"
#include "velocity_estimation.h"
#include "QuadEncoder.h"

class model_encoder
{
public:
    void initialize_encoder(uint8_t rotationalDirection, float offset, float resolution, uint8_t port);

    void reset_encoder(void);

    void position_reset_encoder(float offset);

    float read_encoder_angle(void);

    float get_angle(void);

    void set_parameters(uint8_t direction, float offset, float resolution);

    /**
     * Main logic loop for calculating detected encoder velocity
     *
     * @param currentTime current micros count
     */
    void velocityEstimation(void);

    float getVelocity(void);

private:
    QuadEncoder *_encoder;
    int32_t _offset;
    int32_t _resolution;
    uint8_t _port;
    int32_t _position;
    uint8_t _pinA;
    uint8_t _pinB;

    float _angle;
    float _angularVelocity;
    velocity_estimation _velocityEstimation;
    boolean _is_multi_turn;
    int _turn_count;
    float _last_position;
};

#endif