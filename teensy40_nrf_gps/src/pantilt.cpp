#include <Arduino.h>
#include <Servo.h>

// put function declarations here:
// Servo pitch; //create servo type object
// Servo yaw;

// float pitch_yaw[2] = {10, 10};
// float pitch_yaw_increment[2] = {0, 0};

Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 100; // variable to store the servo position
int fullrotation = 1800;
int last_time = 0;

int turnState = 0;
// bool angle_updated = false; // not used

void pantilt_setup()
{

  last_time = millis();
  myservo.attach(19);

  // pitch.attach(5,300,2700);
  // // pitch.attach(4);
  // yaw.attach(19);
  // pitch.attach(18);

  // //defaul pos
  // yaw.write(30);
  // pitch.write(80);
  // // pitch.write(90);
}

void turnRight()
{
  myservo.writeMicroseconds(1400);
}

void turnLeft()
{
  myservo.writeMicroseconds(1600);
}

void turnStop()
{
  myservo.writeMicroseconds(0);
}

void pantilt_loop()
{

  if (turnState == 0)
  {
    turnStop();
  }
  else if (turnState == 1)
  {
    turnLeft();
    // Serial.println(pos);
  }
  else if (turnState == -1)
  {
    turnRight();
  }

  pos += 10 * turnState;

  if (pos > fullrotation || pos < 0)
  {
    turnState = 0;
    turnStop();
  }

  // while(millis() - last_time < 1);
  // last_time = millis();

  // pitch.write(90);
  // yaw.write(90);

  // pitch.write((int)pitch_yaw[0]);
  // yaw.write((int)pitch_yaw[1]);
}
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
