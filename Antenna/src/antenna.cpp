#include <Arduino.h>

#include "Servo.h"
#include <math.h>
#include <iostream>

#define ANTENNA_PWM_PIN 9

Servo servo;

void antenna_setup(){
    Serial.begin(9600);
    servo.attach(9);
    delay(3000);
    servo.write(90); // default pos
}
 
void antenna_loop(){
// empty
}

void set_pos(float pos){
  servo.write((int) pos);
}