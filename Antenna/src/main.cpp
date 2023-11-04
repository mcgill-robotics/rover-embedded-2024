//#ifdef ANTENNA
//#include "antenna.h"
#include <math.h>
#include <Arduino.h>
#include "Servo.h"

#define ANTENNA_PWM_PIN 1

extern float rover_coords[2]; //latitude, longitude
extern float antenna_heading_params[4]; //latitude, longitude, compass angle
extern float servo_angle[1];

void antenna_setup();
void antenna_loop();

float rover_coords[2]; //latitude, longitude
float antenna_heading_params[4]; // antenna latitude, antenna longitude, rover initial latitude, rover initial longtitude
float servo_angle[1]; // final angle (theta + 90)

Servo servo;

void antenna_setup(){
    servo.attach(ANTENNA_PWM_PIN);
    delay(3000);
}

void antenna_loop(){
  // Calculating the differences
  double new_latitude_diff = rover_coords[0] - antenna_heading_params[0];
  double new_longitude_diff = rover_coords[1] - antenna_heading_params[1];
  double initial_latitude_diff = antenna_heading_params[2] - antenna_heading_params[0];
  double initial_longitude_diff = antenna_heading_params[3] - antenna_heading_params[1];
  double new_distance = sqrt(pow(new_latitude_diff, 2) + pow(new_longitude_diff, 2)); // new distance between the antenna and the rover
  double initial_distance = sqrt(pow(initial_latitude_diff, 2) + pow(initial_longitude_diff, 2)); // initial distance between the antenna and the rover

  // Normalizing the angle
  double dot_product_initial_new_distance_diff = initial_latitude_diff * new_latitude_diff + initial_longitude_diff * new_longitude_diff;
  double divider =  new_distance * initial_distance;
  double theta_deg = 0;
  double sin_theta = 0;
  if(divider > 1e-16){
    double theta_rad = acos(dot_product_initial_new_distance_diff/(divider));
    theta_deg = theta_rad * 180.0/M_PI;
    double cross_product = new_latitude_diff * initial_longitude_diff - initial_latitude_diff * new_longitude_diff;
    sin_theta = cross_product/ divider;

  }

  
  // Adding the offset and setting the servo angle
  if (sin_theta < 0){
    servo_angle[0] = (float)(90 + theta_deg);
    servo.write(servo_angle[0]);
  }
  else{
    servo_angle[0] = (float)(90 - theta_deg);
    servo.write(servo_angle[0]);
  }
}

//#endif