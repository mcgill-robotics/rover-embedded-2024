#include <Arduino.h>
#include <Servo.h>


// put function declarations here:
Servo pitch; //create servo type object
Servo yaw;

float pitch_yaw[2] = {10, 10};
float pitch_yaw_increment[2] = {0, 0};
// bool angle_updated = false; // not used

void pantilt_setup() {
  // pitch.attach(5,300,2700);
  pitch.attach(5, 300, 2700);
  yaw.attach(6,300,2800); 

  //defaul pos
  yaw.write(30);
  pitch.write(90);

}

void pantilt_loop() {

    pitch.write((int)pitch_yaw[0]);
    yaw.write((int)pitch_yaw[1]);

/*
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
*/
}