#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop()
{
  // Serial.print("Loop");
  if (radio.available())
  {
    char combinedCString[32] = "";
    radio.read(&combinedCString, sizeof(combinedCString));
    // std::cout<<combinedCString;
    Serial.print("Loop\r\n");
    Serial.println(combinedCString);
  }
}