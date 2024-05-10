# rover-embedded-2024

# Base Station Antenna

## ROS Topics
- 1. `/antennaGPSData` Float32MultiArray - Publisher
- 3. `/antennaHeadingOverideCmd` Float32MultiArray - Subscriber


## Related Variables
- 1. `antenna_gps_coords[latitude, longitude]`
- 3. `antenna_heading[angle]` - from 0 to 180

## Functionality
This board provides the following functionality:
- Points Base Station Antenna based on given angle
- Return Antenna GPS Position

## Note
During base antenna setup, it is assumed that the motor is at 90 degrees and pointing at the rover
