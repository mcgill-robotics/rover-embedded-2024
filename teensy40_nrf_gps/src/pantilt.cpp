#include <Arduino.h>
#include <Servo.h>


// put function declarations here:
Servo pitch; //create servo type object
Servo yaw;

float pitch_yaw[2] = {0, 0};
float pitch_yaw_increment[2] = {0, 0};
bool angle_updated = false;

void pantilt_setup() {
  // put your setup code here, to run once:
  pitch.attach(5,300,2700);
  yaw.attach(7,300,3100); 

}

void pantilt_loop() {

    pitch.write((int)pitch_yaw[0]);
    yaw.write((int)pitch_yaw[1]);

// //   if (!angle_updated) return;
//     pitch_yaw[1] = 180;//2300;
//     pitch_yaw[0] = 180;//2300;

//   if (pitch_yaw_increment[0] < pitch_yaw[0] && pitch_yaw_increment[0] != pitch_yaw[0] ){
//     pitch_yaw_increment[0] += 1;
//   } else if(pitch_yaw_increment[0] != pitch_yaw[0] ){
//     pitch_yaw_increment[0] -= 1;
//   }

//   if (pitch_yaw_increment[1] < pitch_yaw[1] && pitch_yaw_increment[1] != pitch_yaw[1] ){
//     pitch_yaw_increment[1] += 1;
//   } else if (pitch_yaw_increment[1] != pitch_yaw[1] ){
//     pitch_yaw_increment[1] -= 1;
//   }

//  // pitch.write((int)pitch_yaw_increment[0]); 
//   delay(10); // wait for 500 ms
//   yaw.write((int)pitch_yaw_increment[1]); 
//   delay(10);      

// for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
//   {                                  // in steps of 1 degree 
//     yaw.write(pos);              // tell servo to go to position in variable 'pos' 
//     delay(15);                       // waits 15ms for the servo to reach the position 
//   } 
//   for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
//   {                                
//     yaw.write(pos);              // tell servo to go to position in variable 'pos' 
//     delay(15);                       // waits 15ms for the servo to reach the position 
//   } 

//   angle_updated = false;
}