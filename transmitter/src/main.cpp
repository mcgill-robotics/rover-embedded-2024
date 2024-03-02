// NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>
#include <string>

RF24 radio(7, 8); // CE, CSN pins of NRF
const byte address[6] = "00001";

// moisture sensor(2)
#include <Arduino.h>
#define moisture0 A0 // analog pin on teensy
#define moisture1 A1 // analog pin on teensy

// pH sensor(1)
#define ph0 A2              // analog pin on teensy
unsigned long int avgValue; // average of 10 milli volt readings
int buf[10], temp;          // array of 10 milli volt readings

void setup()
{
    Serial.begin(9600);

    // NRF24
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.stopListening();

    // moisture
    pinMode(moisture0, INPUT);
    pinMode(moisture1, INPUT);

    // pH
    pinMode(ph0, INPUT);
    Serial.begin(9600);
}

void loop()
{
    // moisture
    int sensorValue0 = analogRead(moisture0); // moisture sensor value 1
    int sensorValue1 = analogRead(moisture1); // moisture sensor value 2
    int moistureArray[] = {sensorValue0, sensorValue1, 0, 0};

    // pH
    for (int i = 0; i < 10; i++) // Get 10 sample value from the sensor for smooth the value
    {
        buf[i] = analogRead(ph0);
        delay(10);
    }

    for (int i = 0; i < 9; i++) // sort the analog from small to large
    {
        for (int j = i + 1; j < 10; j++)
        {
            if (buf[i] > buf[j])
            {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    avgValue = 0;
    for (int i = 2; i < 8; i++)
    { // take the average value of 6 center sample
        avgValue += buf[i];
    }
    avgValue = avgValue / 6;

    float milVolt = (float)avgValue * (5.0 / 1023.0); // convert the analog reading into millivolt
    // Serial.println(milVolt);
    float phValue = -7.4074 * milVolt + 22.8333333; // convert the millivolt into pH value

    float pHArray[] = {1.1, 2.2, 3.3, 4.4};

    // Convert integers to strings
    std::string intString1 = std::to_string(moistureArray[0]);
    std::string intString2 = std::to_string(moistureArray[1]);
    std::string intString3 = std::to_string(moistureArray[2]);
    std::string intString4 = std::to_string(moistureArray[3]);

    // Convert floats to strings
    std::string floatString1 = std::to_string(pHArray[0]);
    std::string floatString2 = std::to_string(pHArray[1]);
    std::string floatString3 = std::to_string(pHArray[2]);
    std::string floatString4 = std::to_string(pHArray[3]);

    // Concatenate the strings
    std::string combinedString = intString1 + ", " + intString2 + ", " + intString3 + ", " + intString4 + ", " +
                                 floatString1 + ", " + floatString2 + ", " + floatString3 + ", " + floatString4;

    const char *combinedCString = combinedString.c_str();
    char test[] = "hi";
    // radio.write(&combinedCString, sizeof(combinedCString));
    Serial.printf("TX %d\r\n", sizeof combinedCString);
    // radio.write(&test, strlen(test));
    size_t combinedLength = strlen(combinedCString) + 1;

    radio.write(&combinedCString, combinedLength);
    Serial.printf("sensorValue0=%d, sensorValue1=%d,", sensorValue0, sensorValue1);
    Serial.println();

    delay(1000);
}