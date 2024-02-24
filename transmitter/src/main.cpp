// NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN pins of NRF
const byte address[6] = "00001";

// moisture sensor(2)
#include <Arduino.h>
#define sensor0 A0 // moisture pin
#define sensor1 A1 // moisture pin

// pH sensor(1)
#define SensorPin A2        //analog pin A2 on teensy 4.0
unsigned long int avgValue;  //avgvalue stores the average of 10 read
float b;
int buf[10],temp;


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
    pinMode(A0,INPUT);  
    Serial.begin(9600);  
    Serial.println("Ready");    //Test the serial monitor

}

void loop()
{
    // moisture
    int sensorValue0 = analogRead(sensor0); // moisture sensor value 1
    int sensorValue1 = analogRead(sensor1); // moisture sensor value 2


    // pH
    for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
    { 
        buf[i]=analogRead(A0);
        delay(10);
    }
    for(int i=0;i<9;i++)        //sort the analog from small to large
    {
        for(int j=i+1;j<10;j++)
        {
        if(buf[i]>buf[j])
        {
            temp=buf[i];
            buf[i]=buf[j];
            buf[j]=temp;
        }
        }
    }
    avgValue=0;
    for(int i=2;i<8;i++) {                     //take the average value of 6 center sample
        avgValue+=buf[i];
    }
    avgValue = avgValue/6;

    float milVolt=(float)avgValue*(5.0/1023.0); //convert the analog into millivolt   
    Serial.println(milVolt);
    delay(800);
    
    float phValue=-7.4074*milVolt + 22.8333333;                      //convert the millivolt into pH value
    

    int moisture_output[8] = {sensorValue0, sensorValue1, phValue, 0, 0, 0, 0, 0}; // array of 4 pH values, 4 moisture values

    radio.write(&moisture_output, sizeof(moisture_output));
    delay(1000);
}