#include <Arduino.h>

#include <stdio.h>
#include <iostream>
#include <array>
#include <string>

#define DATA_FREQUENCY 10

// moisture sensor
#define moisture0 A7
#define moisture1 A6
#define moisture2 A5
#define moisture3 A4

// pH sensor
#define ph0 A3
#define ph1 A2
#define ph2 A1
#define ph3 A0

volatile uint32_t geiger_count = 0;
uint32_t last_geiger_count = 0;

uint32_t geiger_buffer[DATA_FREQUENCY] = {0};
uint32_t geiger_index = 0;
float geiger_per_second = 0.0;

void update_pH_data()
{
    for (int i = 0; i < 10; i++)
    { // Get 10 sample value from the sensor for smooth the value
        buf0[i] = analogRead(ph0);
        buf1[i] = analogRead(ph1);
        buf2[i] = analogRead(ph2);
        buf3[i] = analogRead(ph3);
        delay(10);
    }

    for (int i = 0; i < 9; i++)
    { // sort the analog from small to large
        for (int j = i + 1; j < 10; j++)
        {
            if (buf0[i] > buf0[j])
            {
                temp0 = buf0[i];
                buf0[i] = buf0[j];
                buf0[j] = temp0;
            }

            if (buf1[i] > buf1[j])
            {
                temp1 = buf1[i];
                buf1[i] = buf1[j];
                buf1[j] = temp1;
            }

            if (buf2[i] > buf2[j])
            {
                temp2 = buf2[i];
                buf2[i] = buf2[j];
                buf2[j] = temp2;
            }

            if (buf3[i] > buf3[j])
            {
                temp3 = buf3[i];
                buf3[i] = buf3[j];
                buf3[j] = temp3;
            }
        }
    }

    avgValue0 = 0;
    avgValue1 = 0;
    avgValue2 = 0;
    avgValue3 = 0;

    for (int i = 2; i < 8; i++)
    { // take the average value of 6 center sample
        avgValue0 += buf0[i];
        avgValue1 += buf1[i];
        avgValue2 += buf2[i];
        avgValue3 += buf3[i];
    }

    avgValue0 = avgValue0 / 6;
    avgValue1 = avgValue1 / 6;
    avgValue2 = avgValue2 / 6;
    avgValue3 = avgValue3 / 6;

    float milVolt0 = (float)avgValue0 * (5.0 / 1023.0);    // convert the analog reading into millivolt
    float phValue0 = -3.71654359 * milVolt0 + 14.59650933; // convert the millivolt into pH value

    float milVolt1 = (float)avgValue1 * (5.0 / 1023.0);
    float phValue1 = -3.71654359 * milVolt1 + 14.59650933;

    float milVolt2 = (float)avgValue2 * (5.0 / 1023.0);
    float phValue2 = -3.71654359 * milVolt2 + 14.59650933;

    float milVolt3 = (float)avgValue3 * (5.0 / 1023.0);
    float phValue3 = -3.71654359 * milVolt3 + 14.59650933;

    transmitter_data[4] = phValue0;
    transmitter_data[5] = phValue1;
    transmitter_data[6] = phValue2;
    transmitter_data[7] = phValue3;
}

void update_geiger()
{
    uint32_t geiger_clicks = geiger_count - last_geiger_count;
    geiger_buffer[geiger_index++] = geiger_clicks;
    geiger_index = geiger_index % DATA_FREQUENCY;
    geiger_per_second = 0;
    for (int i = 0; i < DATA_FREQUENCY; i++)
    {
        geiger_per_second += geiger_buffer[i];
    }

    last_geiger_count = geiger_count;
}

void extGeiger_INT()
{
    geiger_count += 1;
}