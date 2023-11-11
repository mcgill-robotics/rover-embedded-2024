/**
 * @file          hardware_pins.h
 * @author        Steve Ding, Oliver Philbin-Briscoe, Colin Gallacher
 * @version       V0.1.0
 * @date          11-March-2021
 * @brief         hardware pin definitions
 *
 * @attention     For prototype example only
 */

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

#include <stdint.h>

const uint8_t ENCPIN1_1 = 3;
const uint8_t ENCPIN1_2 = 4;

const uint8_t ENCPIN2_1 = 7;
const uint8_t ENCPIN2_2 = 8;

const uint8_t ENCPIN3_1 = 31;
const uint8_t ENCPIN3_2 = 30;


const uint8_t PWMPIN1 = 0;  // enable pin
const uint8_t DIRPIN1 = 2; // phase pin
const uint8_t nSLEEP1 = 1;

const uint8_t PWMPIN2 = 10;
const uint8_t DIRPIN2 = 12;
const uint8_t nSLEEP2 = 11;

const uint8_t PWMPIN3 = 28;
const uint8_t DIRPIN3 = 32;
const uint8_t nSLEEP3 = 29;


const uint8_t LED_O = 34;
const uint8_t LED_B = 33;


const uint8_t CURRENT_SENSE_A = 23;

const uint8_t CURRENT_SENSE_B = 24;

const uint8_t CURRENT_SENSE_C = 25;
#endif