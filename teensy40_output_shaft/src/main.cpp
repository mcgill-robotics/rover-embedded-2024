// #include <Arduino.h>
// #include <SPI.h>
// #include <EEPROM.h>
// #include "AMT22.h"     // Assuming this is your custom library for the AMT22 encoder.
// #include "rover_arm.h" // Make sure this contains necessary definitions for your project.

// #define MIN_ADC_VALUE 0
// #define MAX_ADC_VALUE 4095
// #define POLL_DELAY_MS 1000

// struct Joint
// {
//   float angle_continuous;
//   int error;
// };

// Joint elbow, shoulder, waist;

// float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
// {
//   return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
// }

// void read_joint_angle_multi(Joint &joint, uint8_t CS_pin)
// {
//   int16_t result_arr[2];
//   int error = getTurnCounterSPI(result_arr, CS_pin, 12);

//   if (error == -1)
//   {
//     joint.error = error;
//   }
//   else
//   {
//     float angle_deg = mapFloat((float)result_arr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
//     float turns = result_arr[1];
//     joint.angle_continuous = angle_deg + 360 * turns;
//     joint.error = 0;
//   }
// }

// void setup()
// {
//   Serial.begin(115200);
//   SPI.begin();
//   SPI.setClockDivider(SPI_CLOCK_DIV64);
//   pinMode(CS1, OUTPUT);
//   pinMode(CS2, OUTPUT);
//   pinMode(CS3, OUTPUT);
// }

// void loop()
// {
//   delay(POLL_DELAY_MS);

//   read_joint_angle_multi(elbow, CS1);
//   read_joint_angle_multi(shoulder, CS2);
//   read_joint_angle_multi(waist, CS3);

//   Serial.printf("Elbow: %s, Shoulder: %s, Waist: %s\n",
//                 elbow.error == -1 ? "Error" : String(elbow.angle_continuous, 2).c_str(),
//                 shoulder.error == -1 ? "Error" : String(shoulder.angle_continuous, 2).c_str(),
//                 waist.error == -1 ? "Error" : String(waist.angle_continuous, 2).c_str());
// }
