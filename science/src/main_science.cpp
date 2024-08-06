#include "common.h"

#include <Arduino.h>
#include <ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

#define DEBUG_PRINT 1

RF24 radio(10, 6); // CE, CSN pins of NRF
const byte address[6] = "00001";
float transmitter_data[8];

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
unsigned long int avgValue0; // average of 10 milli volt readings
unsigned long int avgValue1;
unsigned long int avgValue2;
unsigned long int avgValue3;
int buf0[10], temp0; // array of 10 milli volt readings for pH 0
int buf1[10], temp1; // array of 10 milli volt readings for pH 1
int buf2[10], temp2; // array of 10 milli volt readings for pH 2
int buf3[10], temp3; // array of 10 milli volt readings for pH 3
uint32_t lastTime = 0;

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

void update_moisture_data()
{
    int sensorValue0 = analogRead(moisture0);
    int sensorValue1 = analogRead(moisture1);
    int sensorValue2 = analogRead(moisture2);
    int sensorValue3 = analogRead(moisture3);

    transmitter_data[0] = sensorValue0;
    transmitter_data[1] = sensorValue1;
    transmitter_data[2] = sensorValue2;
    transmitter_data[3] = sensorValue3;
}

void send_payload_rf24(uint8_t *data, size_t length)
{
    bool result = radio.write(data, length);
    Serial.printf("RESULT=%d, Sending %d bytes: ", result, length);
    for (uint32_t i = 0; i < length; i++)
    {
        // Print byte in XX:XX format
        Serial.printf("%02X", data[i] & 0xFF);
        Serial.print(":");
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    // NRF24
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.stopListening();

    // moisture
    pinMode(moisture0, INPUT);
    pinMode(moisture1, INPUT);
    pinMode(moisture2, INPUT);
    pinMode(moisture3, INPUT);

    // pH
    pinMode(ph0, INPUT);
    pinMode(ph1, INPUT);
    pinMode(ph2, INPUT);
    pinMode(ph3, INPUT);

    lastTime = millis();
}

void loop()
{
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);

    update_moisture_data();
    update_pH_data();
    if (millis() - lastTime > 100)
    {
        lastTime = millis();
        send_payload_rf24((uint8_t *)transmitter_data, sizeof(transmitter_data));
        Serial.printf("pH data: %f, %f, %f, %f\r\n", transmitter_data[0], transmitter_data[1], transmitter_data[2], transmitter_data[3]);
        Serial.printf("moisture data: %f, %f, %f, %f\r\n", transmitter_data[4], transmitter_data[5], transmitter_data[6], transmitter_data[7]);
        Serial.printf("sizeof float: %d\r\n", sizeof(float));
        Serial.printf("sizeof transmitter_data: %d\r\n", sizeof(transmitter_data));
        Serial.printf("sizeof transmitter_data[0]: %d\r\n", sizeof(transmitter_data[0]));
        lastTime = millis();
    }
}