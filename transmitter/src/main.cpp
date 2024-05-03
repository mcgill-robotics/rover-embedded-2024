// NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>
#include <string>

#define DEBUG_PRINT 1

RF24 radio(7, 8); // CE, CSN pins of NRF
const byte address[6] = "00001";

#include <Arduino.h>

// moisture sensor
#define moisture0 A0 // analog pin on teensy
#define moisture1 A1 // analog pin on teensy

// pH sensor
#define ph0 A2              // analog pin on teensy
#define ph1 A3
unsigned long int avgValue0; // average of 10 milli volt readings
unsigned long int avgValue1;
int buf0[10], temp;          // array of 10 milli volt readings
int buf1[10], temp;
uint32_t lastTime = 0;

float pH_data[] = {1.1, 2.2, 3.3, 4.4};
int moisture_data[] = {1, 2, 3, 4};

void update_pH_data()
{
    // pH
    for (int i = 0; i < 10; i++) // Get 10 sample value from the sensor for smooth the value
    {
        buf0[i] = analogRead(ph0);
        buf1[i] = analogRead(ph1);
        delay(10);
    }

    for (int i = 0; i < 9; i++) // sort the analog from small to large
    {
        for (int j = i + 1; j < 10; j++)
        {
            if (buf0[i] > buf0[j])
            {
                temp = buf0[i];
                buf0[i] = buf0[j];
                buf0[j] = temp;
            }

            if (buf1[i] > buf1[j])
            {
                temp = buf1[i];
                buf1[i] = buf1[j];
                buf1[j] = temp;
            }
        }
    }

    avgValue0 = 0;
    avgValue1 = 0;
    for (int i = 2; i < 8; i++)
    { // take the average value of 6 center sample
        avgValue0 += buf0[i];
        avgValue1 += buf1[i];
    }
    avgValue0 = avgValue0 / 6;
    avgValue1 = avgValue1 / 6;

    float milVolt0 = (float)avgValue0 * (5.0 / 1023.0); // convert the analog reading into millivolt
    float phValue0 = -3.71654359 * milVolt0 + 14.59650933;   // convert the millivolt into pH value

    float milVolt1 = (float)avgValue1 * (5.0 / 1023.0);
    float phValue1 = -3.71654359 * milVolt1 + 14.59650933; 

    pH_data[0] = phValue0;
    pH_data[1] = phValue1;
    pH_data[2] = 0;
    pH_data[3] = 0;

    // FOR DEBUG, REMOVE LATER (TODO)
    for (int i = 0; i < 3; i++)
    {
        // pH_data[i] = pH_data[i] % 7 + 0.1;
        // float mod
        pH_data[i] = fmod(pH_data[i] + phValue0 + 0.1, 7);
    }
}

void update_moisture_data() {
    int sensorValue0 = analogRead(moisture0); // moisture sensor value 1
    int sensorValue1 = analogRead(moisture1); // moisture sensor value 2
    
    moisture_data[0] = sensorValue0;
    moisture_data[1] = sensorValue1;
    moisture_data[2] = 0;
    moisture_data[3] = 0;
}

void setup()
{
    Serial.begin(9600);

    // NRF24
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX);
    radio.stopListening();

    // moisture
    pinMode(moisture0, INPUT);
    pinMode(moisture1, INPUT);

    // pH
    pinMode(ph0, INPUT);
    Serial.begin(9600);
    lastTime = millis();
}

void loop()
{
#if DEBUG_PRINT == 1
    if (millis() - lastTime > 1000)
    {
        Serial.println("TX Loop");
        lastTime = millis();
    }
#endif

    update_moisture_data();
    update_pH_data();

    char combinedBuffer[256]; // Adjust the size based on the expected output length.

    int length = snprintf(combinedBuffer, sizeof(combinedBuffer),
                          "%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f",
                          moisture_data[0], moisture_data[1], moisture_data[2], moisture_data[3],
                          pH_data[0], pH_data[1], pH_data[2], pH_data[3]);

    radio.write(combinedBuffer, length + 1); // Include the null terminator in the length

    // DEBUG
    Serial.printf("Buffer: %s\n", combinedBuffer);

    Serial.printf("moisture_data[0]=%d, moisture_data[1]=%d, moisture_data[2]=%d, moisture_data[3]=%d\n",
                  moisture_data[0], moisture_data[1], moisture_data[2], moisture_data[3]);
    Serial.println();

    delay(10);
}