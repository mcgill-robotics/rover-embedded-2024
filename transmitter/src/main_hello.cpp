// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>

// RF24 radio(10, 6); // CE, CSN pins of NRF

// const byte address[6] = "00001";

// void setup()
// {
//     radio.begin();
//     radio.openWritingPipe(address);
//     radio.setPALevel(RF24_PA_HIGH);
//     radio.stopListening();
// }

// void loop()
// {
//     Serial.println("Sending...");
//     const char text[] = "Hello World";
//     radio.write(&text, sizeof(text));
//     delay(500);
// }