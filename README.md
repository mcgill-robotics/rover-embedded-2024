# rover-embedded-2024

# Rover Science Box

## ROS Topics
- `/science_data` publishes a Float32MultiArray of length 12 which is data from cuvettes
- `/stepper_position` publishes a Float32MultiArray of length 1 which is the updated carousel position
- `/augerCmd` subscribes to a Float32MultiArray of length 2 which controls the screw(up/down) and auger(soil collection) respectively. values can be -1(down), 0(stop), 1(up)
- `/stepperCmd` subscribes to a Float32MultiArray of length 1 which prompts the carousel to turn 1/8 of a rotation

## Related Variables
- `stepper_pos` Positions 0/2/4/6 are diagonal(aligned with geiger), positions 1/3/5/7 are straight
- `science_data_msg` elements 0-3 are pH values, 4-7 are moisture values, 8-11 are geiger values

## Functionality
This board provides the following functionality:
- Returns data from 4 cuvettes as a Float32MultiArray over ROSSerial
- Accepts commands to control 2 Auger motors
- Accepts commands to control 1 Carousel/Stepper motor, and publishes the updated Carousel position
