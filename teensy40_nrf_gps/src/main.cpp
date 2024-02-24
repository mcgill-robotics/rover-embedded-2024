#include <Arduino.h>

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GPSBaud = 9600;
extern nrf24_setup();
extern nrf24_loop();
extern void gps_setup();
extern void gps_loop();
// The TinyGPSPlus object
// TinyGPSPlus gps;
// // The serial connection to the GPS device
// static const int RXPin = 4, TXPin = 3;
// SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);
  while (!Serial)
    ;

  gps_setup();
  nrf24_setup();
}

void loop()
{
  gps_loop();
  nrf24_loop();
}
