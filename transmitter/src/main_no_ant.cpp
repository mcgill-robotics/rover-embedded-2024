#include <Arduino.h>

#include <stdio.h>
#include <iostream>
#include <array>
#include <string>

#define DATA_FREQUENCY 10 // MINIMUM IS 1
#define DATA_FREQUENCY_MS 1000 / DATA_FREQUENCY
#define PH_MOISTURE_SAMPLES_PER_SECOND 100
#define PH_MOISTURE_SAMPLES_MS 1000 / PH_MOISTURE_SAMPLES_PER_SECOND

// moisture sensor
#define MOISTURE1 21
#define MOISTURE2 19
#define MOISTURE3 17
#define MOISTURE4 15

// pH sensor
#define PH1 20
#define PH2 18
#define PH3 16
#define PH4 14

// geiger
#define GEIGER_INT_PIN 10

volatile uint32_t geiger_count = 0;
uint32_t last_geiger_count = 0;

uint32_t geiger_buffer[DATA_FREQUENCY] = {0};
uint32_t geiger_index = 0;
float geiger_per_second = 0.0;

volatile uint16_t ph1_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t ph1_buffer_index = 0;
float ph1 = 7.0;

volatile uint16_t ph2_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t ph2_buffer_index = 0;
float ph2 = 7.0;

volatile uint16_t ph3_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t ph3_buffer_index = 0;
float ph3 = 7.0;

volatile uint16_t ph4_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t ph4_buffer_index = 0;
float ph4 = 7.0;

volatile uint16_t moisture1_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t moisture1_buffer_index = 0;
float moisture1 = 0.0;

volatile uint16_t moisture2_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t moisture2_buffer_index = 0;
float moisture2 = 0.0;

volatile uint16_t moisture3_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t moisture3_buffer_index = 0;
float moisture3 = 0.0;

volatile uint16_t moisture4_buffer[PH_MOISTURE_SAMPLES_PER_SECOND];
volatile uint16_t moisture4_buffer_index = 0;
float moisture4 = 0.0;

IntervalTimer phMoistureTimer;

float message_data[9];
char message_bytes[38];

void extGeiger_INT();
void extPhMoisture_INT();
void update_pH_moisture();
void update_geiger();

void setup()
{

    pinMode(PH1, INPUT);
    pinMode(PH2, INPUT);
    pinMode(PH3, INPUT);
    pinMode(PH4, INPUT);

    pinMode(MOISTURE1, INPUT);
    pinMode(MOISTURE2, INPUT);
    pinMode(MOISTURE3, INPUT);
    pinMode(MOISTURE4, INPUT);

    pinMode(GEIGER_INT_PIN, INPUT);

    analogReadResolution(12);

    phMoistureTimer.begin(extPhMoisture_INT, PH_MOISTURE_SAMPLES_MS * 1000);
    attachInterrupt(GEIGER_INT_PIN, extGeiger_INT, RISING);
    message_bytes[0] = '$';
    message_bytes[1] = '$';
}

void loop()
{
    update_pH_moisture();
    update_geiger();

    message_data[0] = ph1;
    message_data[1] = ph2;
    message_data[2] = ph3;
    message_data[3] = ph4;

    message_data[4] = moisture1;
    message_data[5] = moisture2;
    message_data[6] = moisture3;
    message_data[7] = moisture4;

    message_data[8] = geiger_per_second;

    // memcpy(message_bytes + 2, message_data, 4 * 9);

    // SerialUSB.write(message_bytes, 38);

    for (int i = 0; i < 8; i++)
    {
        Serial.print(message_data[i]);
        Serial.print(',');
    }
    Serial.println(message_data[8]);

    delay(DATA_FREQUENCY_MS);
}

void update_pH_moisture()
{

    // PH
    uint32_t totalSamples1 = 0;
    uint32_t totalSamples2 = 0;
    uint32_t totalSamples3 = 0;
    uint32_t totalSamples4 = 0;
    for (int i = 0; i < PH_MOISTURE_SAMPLES_PER_SECOND; i++)
    {
        totalSamples1 += ph1_buffer[i];
        totalSamples2 += ph2_buffer[i];
        totalSamples3 += ph3_buffer[i];
        totalSamples4 += ph4_buffer[i];
    }

    ph1 = (((float)totalSamples1 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 3.3; // voltages
    ph2 = (((float)totalSamples2 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 3.3; // voltages
    ph3 = (((float)totalSamples3 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 3.3; // voltages
    ph4 = (((float)totalSamples4 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 3.3; // voltages

    ph1 = (-5.6548 * ph1) + 15.509; // PH
    ph2 = (-5.6548 * ph2) + 15.509;
    ph3 = (-5.6548 * ph3) + 15.509;
    ph4 = (-5.6548 * ph4) + 15.509;

    // MOISTURE
    totalSamples1 = 0;
    totalSamples2 = 0;
    totalSamples3 = 0;
    totalSamples4 = 0;
    for (int i = 0; i < PH_MOISTURE_SAMPLES_PER_SECOND; i++)
    {
        totalSamples1 += moisture1_buffer[i];
        totalSamples2 += moisture2_buffer[i];
        totalSamples3 += moisture3_buffer[i];
        totalSamples4 += moisture4_buffer[i];
    }

    moisture1 = ((((float)totalSamples1 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 100.0); // moisture percentages
    moisture2 = ((((float)totalSamples2 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 100.0);
    moisture3 = ((((float)totalSamples3 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 100.0);
    moisture4 = ((((float)totalSamples4 / PH_MOISTURE_SAMPLES_PER_SECOND) / 4095.0) * 100.0);
}

void update_geiger()
{
    uint32_t geiger_clicks = geiger_count - last_geiger_count;
    geiger_buffer[geiger_index++] = geiger_clicks;
    geiger_index %= DATA_FREQUENCY;
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

void extPhMoisture_INT()
{
    ph1_buffer[ph1_buffer_index++] = analogRead(PH1);
    ph2_buffer[ph2_buffer_index++] = analogRead(PH2);
    ph3_buffer[ph3_buffer_index++] = analogRead(PH3);
    ph4_buffer[ph4_buffer_index++] = analogRead(PH4);

    moisture1_buffer[moisture1_buffer_index++] = analogRead(MOISTURE1);
    moisture2_buffer[moisture2_buffer_index++] = analogRead(MOISTURE2);
    moisture3_buffer[moisture3_buffer_index++] = analogRead(MOISTURE3);
    moisture4_buffer[moisture4_buffer_index++] = analogRead(MOISTURE4);

    ph1_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
    ph2_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
    ph3_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
    ph4_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;

    moisture1_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
    moisture2_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
    moisture3_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
    moisture4_buffer_index %= PH_MOISTURE_SAMPLES_PER_SECOND;
}