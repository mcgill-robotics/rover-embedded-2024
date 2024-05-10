#include <Arduino.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


extern float base_gps_coords[2] = {0,0};

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
void displayInfo();

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
// SoftwareSerial ss(RXPin, TXPin);
 
void gps_setup()
{
    // Empty
}

void gps_loop()
{
    // This displays information every time a new sentence is correctly encoded.
    
    while (Serial1.available() > 0)
    {
        if (gps.encode(Serial1.read()))
        {
            displayInfo(); //For Debugging
            
            //--------------------
            float lat = (float)gps.location.lat();
            float lng = (float)gps.location.lng();
            base_gps_coords[0]=lat;
            base_gps_coords[1]=lng;
        }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        base_gps_coords[0] = 0;
        base_gps_coords[1] = 0; // gps returning 0,0 should be seen as an error code
        while (true); // program gives up on you
    }
}

// Displayed Latitude, Longitude, Date, Time
void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}