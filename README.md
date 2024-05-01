# rover-embedded-2024

/*
Board: Rover GPS + Pan Tilt Camera Control
ROS Topics: /roverGPSData
Related Vars: rover_gps_coords[latitude, longitude]

Functionality: Returns current Rover coordinates (every loop) over ROSSerial. Takes in yaw and pitch angles to control camera pan tilt movement.
Note: A GPS value of [0,0] should be treated as an error code
*/
