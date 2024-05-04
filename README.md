# rover-embedded-2024

# Base Station Antenna

## ROS Topics
- 1. `/antennaGPSData`
- 2. `/antennaGPSOverideCmd`
- 3. `/antennaHeadingOverideCmd`
- 4. `/roverGPSFeedCmd`

## Related Variables
- 1. `antenna_gps_coords[latitude, longitude]`
- 2. `antenna_gps_coords[latitude, longitude]`
- 3. `antenna_heading[x, y]` - set it to 1,1
- 4. `rover_gps_coords[latitude, longitude]`

## Functionality
This board provides the following functionality:
- Points Base Station Antenna Towards Rover based on given rover gps data
- Return Antenna GPS Position

## Note
During base antenna setup, it is assumed that the motor is at 90 degrees and pointing at the rover
