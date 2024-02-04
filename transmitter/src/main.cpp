// NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// moisture sensor(2)
#include <Arduino.h>
#define sensor0 A0
#define sensor1 A1

// pH sensor(1)
// #define SensorPin A2        // input pin for pH sensor
// unsigned long int avgValue;  // sensor output average value
// float b;
// int buf[10],temp;


void setup()
{
    Serial.begin(9600);

    // NRF24
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.stopListening();

    // moisture
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    // pH
    // pinMode(13,OUTPUT);    

}

void loop()
{
    // moisture
    int sensorValue0 = analogRead(sensor0); // moisture sensor value 1
    int sensorValue1 = analogRead(sensor1); // moisture sensor value 2


    // // pH
    // for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
    // { 
    //     buf[i]=analogRead(SensorPin);
    //     delay(10);
    // }

    // for(int i=0;i<9;i++)        //sort the analog from small to large
    // {
    //     for(int j=i+1;j<10;j++)
    //     {
    //         if(buf[i]>buf[j])
    //         {
    //             temp=buf[i];
    //             buf[i]=buf[j];
    //             buf[j]=temp;
    //         }
    //     }
    // }
    // avgValue=0;
    // for(int i=2;i<8;i++) {        
    //     avgValue+=buf[i];
    // }
    // avgValue = avgValue/6;  //average value of 6 center sample(raw analog value)

    // NRF24
    // char text[500];
    // sprintf(text, "Sensor 0: %d, Sensor 1: %d, Hello World", sensorValue0, sensorValue1);
    // radio.write(&text, sizeof(text));
    // delay(1000);


    int text[8] = {sensorValue0, sensorValue1, 0, 0, 0, 0, 0, 0}; // text is array: 4 pH values, 4 moisture values

    radio.write(&text, sizeof(text));
    delay(1000);
}