#include <Arduino.h>
// #include <ros.h> 

// using namespace std;


extern void antenna_setup();
extern void antenna_loop();


void setup(){
  antenna_setup();
}


void loop(){
  antenna_loop();
}
