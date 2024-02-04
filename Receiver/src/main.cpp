#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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
  if (radio.available())
  {
    int text[8];
    radio.read(&text, sizeof(text)); //NRF receives array of 8 integers

    String output_int = "";

    // Concatenate each element of the array to the string
    for (int i = 0; i < sizeof(text) / sizeof(text[0]); ++i) {
        output_int += String(text[i]) + " ";
    }

    // prints 8 integers: 4 for pH value, 4 for moisture value
    Serial.println(output_int);
    }
}