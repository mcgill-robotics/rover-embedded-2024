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
#include "QuadEncoder.h"

class model_encoder
{
public:
    QuadEncoder *_encoder;

    void initialize_encoder(uint8_t rotationalDirection, float offset, float resolution, uint8_t port);

    void reset_encoder(void);

    void poll_encoder_angle(void);

    float get_angle(void);

    void set_parameters(uint8_t direction, float offset, float resolution);

    /**
     * Main logic loop for calculating detected encoder velocity
     *
     * @param currentTime current micros count
     */

private:
    // QuadEncoder *_encoder;
    int32_t _offset;
    int32_t _resolution;
    uint8_t _port;
    int32_t _quad_enc_pos;
    uint8_t _pinA;
    uint8_t _pinB;

    float _angle;
    float _last_angle;
};

#endif