#include <Arduino.h>
#define SensorPin A0        //analog pin A0 on teensy 4.0
unsigned long int avgValue;  //avgvalue stores the average of 10 read
float b;
int buf[10],temp;
 
void setup()
{
  pinMode(A0,INPUT);  
  Serial.begin(9600);  
  Serial.println("Ready");    //Test the serial monitor
}
void loop()
{
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

  Serial.print("    pH:");  
  Serial.print(phValue,2);
  Serial.println(" "); 
  digitalWrite(13, HIGH);       
  delay(800);
  digitalWrite(13, LOW); 
  

}