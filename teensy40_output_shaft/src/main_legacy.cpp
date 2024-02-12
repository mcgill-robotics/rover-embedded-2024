// #include <Arduino.h>

// #include "AMT22.h"
// #include <SPI.h>
// #include <EEPROM.h>
// #include "rover_arm.h"

// #define MIN_ADC_VALUE 0
// #define MAX_ADC_VALUE 4095

// float waist_angle = 0;
// float shoulder_angle = 0;
// float elbow_angle = 0;

// float map_float(float x, float in_min, float in_max, float out_min, float out_max)
// {
//   double result = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

//   return result;
// }

// void setup()
// {
//   // Initiate SPI bus.
//   SPI.begin();
//   SPI.setClockDivider(SPI_CLOCK_DIV64);
//   pinMode(CS1, OUTPUT);
//   pinMode(CS2, OUTPUT);
//   pinMode(CS3, OUTPUT);
// }

// void loop()
// {
//   int16_t result_arr[2];
//   int error = getTurnCounterSPI(result_arr, CS1, 12);
//   if (error == -1)
//   {
//     Serial.println("Error reading SPI.");
//   }
//   else
//   {
//     float angle_raw = map_float((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
//     float turns = result_arr[1];
//     waist_angle = angle_raw + 360 * turns;

//     Serial.printf("Angle: %d, Turns: %d ", result_arr[0], result_arr[1]);
//     Serial.printf("Angle_continous: %f", waist_angle);
//     Serial.println();
//   }
// }