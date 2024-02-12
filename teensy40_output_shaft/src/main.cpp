#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include "AMT22.h"     // Assuming this is your custom library for the AMT22 encoder.
#include "rover_arm.h" // Make sure this contains necessary definitions for your project.

#define MIN_ADC_VALUE 0
#define MAX_ADC_VALUE 4095
#define POLL_DELAY_MS 1000

struct Joint
{
  float angle_continuous;
  int error;
};

Joint waist, shoulder, elbow;

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void read_joint_angle(Joint &joint, uint8_t CS_pin)
{
  int16_t resultArr[2];
  int error = getTurnCounterSPI(resultArr, CS_pin, 12);

  if (error == -1)
  {
    joint.error = error;
  }
  else
  {
    float angleRaw = mapFloat((float)resultArr[0], MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 359.99f);
    float turns = resultArr[1];
    joint.angle_continuous = angleRaw + 360 * turns;
    joint.error = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(CS3, OUTPUT);
}

void loop()
{
  delay(POLL_DELAY_MS);

  read_joint_angle(waist, CS1);
  read_joint_angle(shoulder, CS2);
  read_joint_angle(elbow, CS3);

  Serial.printf("Waist: %s, Shoulder: %s, Elbow: %s\n",
                waist.error == -1 ? "Error" : String(waist.angle_continuous, 2).c_str(),
                shoulder.error == -1 ? "Error" : String(shoulder.angle_continuous, 2).c_str(),
                elbow.error == -1 ? "Error" : String(elbow.angle_continuous, 2).c_str());
}
