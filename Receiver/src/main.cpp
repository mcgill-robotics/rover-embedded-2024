#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

#define DEBUG_PRINT 1

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
uint32_t lastTime = 0;

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
  lastTime = millis();
}

void loop()
{
#if DEBUG_PRINT == 1
  if (millis() - lastTime > 1000)
  {
    Serial.printf("RX Loop %lu\r\n", millis()); // Use %lu for millis() as it returns unsigned long
    lastTime = millis();
  }
#endif
  bool rx_flag = radio.available();
  if (rx_flag)
  {
    char rx_buffer[256];
    uint8_t len = radio.getDynamicPayloadSize(); // Get dynamic payload size
    if (len > 255)
      len = 255; // Ensure len does not exceed buffer size - 1 (for null terminator)
    radio.read(rx_buffer, len);
    rx_buffer[len] = '\0'; // Null-terminate the received string
    Serial.println("Received:");
    Serial.println(rx_buffer);
    Serial.printf("time: %lu\r\n", millis());
  }
}
