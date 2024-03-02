#include <Arduino.h>
#define sensor0 A0
#define sensor1 A1

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue0 = analogRead(sensor0);
  int sensorValue1 = analogRead(sensor1);

  // print out the value you read:
  Serial.print("Sensor 0 value is: ");
  Serial.println(sensorValue0);
  Serial.print("Sensor 1 value is: ");
  Serial.println(sensorValue1);
  delay(1000);        // delay in between reads for stability
}
