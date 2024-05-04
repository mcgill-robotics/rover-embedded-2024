# rover-embedded-2024

# Rover GPS + Pan Tilt Camera Control Board

## ROS Topics
- `/roverGPSData` Float32MultiArray - Publisher
- `/pantiltCmd` Float32MultiArray - Subscriber

## Related Variables
- `rover_gps_coords[latitude, longitude]`
- `pantilt_cmd_cb[pitch, yaw]`

## Functionality
This board provides the following functionality:
- Returns current Rover coordinates (every loop) over ROSSerial.
- Accepts yaw and pitch angles to control camera pan tilt movement.

## Note
A GPS value of [0,0] should be treated as an error code.

![camera rotation](https://i.stack.imgur.com/1OFpX.png)
