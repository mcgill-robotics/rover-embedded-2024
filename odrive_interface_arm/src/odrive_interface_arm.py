import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

# TODO: Figure out why catkin on the Jetson isn't playing nice with this import. It worked on my PC -Eren
# import init_functions
import rospy
from odrive.enums import AxisState, ProcedureResult
from odrive.utils import dump_errors
from std_msgs.msg import Float32MultiArray
from ODrive_utils import *
from ODrive_Joint import *


# TODO find more serial, it is a string of hex of the serial number
arm_serial_numbers = {
    "rover_arm_shoulder": "386434413539",  # 0x386434413539 = 62003024573753 in decimal
    "rover_arm_elbow": "0",
    "rover_arm_waist": "0",
}


# TODO: Once drive is working well, expand this node to include the three arm motors
class Node_odrive_interface_arm:
    def __init__(self):
        # Feedback publishers
        self.elbow_pos_outshaft = 0.0
        self.shoulder_pos_outshaft = 0.0
        self.waist_pos_outshaft = 0.0

        # Setpoints for the arm motors
        self.elbow_pos_setpoint = 0.0
        self.shoulder_pos_setpoint = 0.0
        self.waist_pos_setpoint = 0.0

        # Subscriptions
        rospy.init_node("odrive_interface")
        self.position_subscriber = rospy.Subscriber(
            "/arm_lower_output_shaft_position", Float32MultiArray, self.handle_arm_position_fb
        )
        self.command_subscriber = rospy.Subscriber(
            "/arm_lower_command", Float32MultiArray, self.handle_arm_command
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # From external encoders
    def handle_arm_position_fb(self, command):
        self.elbow_pos_outshaft = command.data[0]
        self.shoulder_pos_outshaft = command.data[1]
        self.waist_pos_outshaft = command.data[2]

    # From the external control node
    def handle_arm_command(self, command):
        self.elbow_pos_setpoint = command.data[0]
        self.shoulder_pos_setpoint = command.data[1]
        self.waist_pos_setpoint = command.data[2]

    def run(self):
        # Discover motors on startup and assign variables for each
        # TODO Elbow and waist motors
        odrv_shoulder = ODrive_Joint(
            odrive.find_any(
                serial_number=arm_serial_numbers["rover_arm_shoulder"], timeout=5
            )
        )

        # MAIN LOOP
        while not rospy.is_shutdown():
            # Apply setpoint
            # TODO Elbow and waist motors
            shoulder_pos_error = self.shoulder_pos_setpoint - self.shoulder_pos_fb
            odrv_shoulder.odrv.axis0.controller.input_pos = self.shoulder_pos_setpoint


            # Get feedback and publish it to "/wheel_velocity_feedback"
            feedback = WheelSpeed()
            measured_speed_lb = -drive_lb.encoder_estimator0.vel_estimate
            measured_speed_lf = -drive_lf.encoder_estimator0.vel_estimate
            measured_speed_rb = drive_rb.encoder_estimator0.vel_estimate
            measured_speed_rf = drive_rf.encoder_estimator0.vel_estimate

            feedback.left[0], feedback.left[1] = measured_speed_lb, measured_speed_lf
            feedback.right[0], feedback.right[1] = measured_speed_rb, measured_speed_rf
            self.feedback_publisher.publish(feedback)

            print(
                f"\rDRIVE_LB: {round(measured_speed_lb, 2)}, DRIVE_LF: {round(measured_speed_lf, 2)}, \
                  DRIVE_RB: {round(measured_speed_rb, 2)}, DRIVE_RF: {round(measured_speed_rf, 2)}",
                end="",
            )

            # Poll the ODrives for their states and check for errors
            for motor in drive_motors:
                state_fb = MotorState()
                state_fb.id = drive_ids[format(motor.serial_number, "x").upper()]

                # TODO: Decode and add state information
                # state_fb.state =
                self.state_publisher.publish(state_fb)

                if motor.axis0.active_errors != 0:
                    # Tell the rover to stop
                    for motor in drive_motors:
                        motor.axis0.controller.input_vel = 0

                    # Wait until it actually stops
                    motor_stopped = False
                    while not motor_stopped:
                        motor_stopped = True
                        for motor in drive_motors:
                            if abs(motor.encoder_estimator0.vel_estimate) >= 0.01:
                                motor_stopped = False
                        if rospy.is_shutdown():
                            print(
                                "Shutdown prompt received. Setting all motors to idle state."
                            )
                            for motor in drive_motors:
                                motor.axis0.requested_state = AxisState.IDLE
                            break

                    # Wait for two seconds while all the transient currents and voltages calm down
                    rospy.sleep(5)

                    # Now try to recover from the error. This will always succeed the first time, but if
                    # the error persists, the ODrive will not allow the transition to closed loop control, and
                    # re-throw the error.
                    motor.clear_errors()
                    rospy.sleep(0.5)
                    motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                    rospy.sleep(0.5)

                    # If the motor cannot recover successfully publish a message about the error, then print to console
                    if motor.axis0.active_errors != 0:
                        error_fb = MotorError()
                        error_fb.id = drive_ids[
                            format(motor.serial_number, "x").upper()
                        ]
                        error_fb.error = decode_errors(motor.axis0.active_errors)
                        self.error_publisher.publish(error_fb)
                        print(
                            f"\nError(s) occurred. Motor ID: {error_fb.id}, Error(s): {error_fb.error}"
                        )

                        # Finally, hang the node and keep trying to recover until the error is gone or the shutdown signal is received
                        print(
                            f"\nMotor {error_fb.id} could not recover from error(s) {error_fb.error}. R to retry, keyboard interrupt to shut down node."
                        )
                        while not rospy.is_shutdown():
                            prompt = input(">").upper()
                            if prompt == "R":
                                motor.clear_errors()
                                rospy.sleep(0.5)
                                motor.axis0.requested_state = (
                                    AxisState.CLOSED_LOOP_CONTROL
                                )
                                rospy.sleep(0.5)
                                if motor.axis0.active_errors == 0:
                                    break
                                else:
                                    print("Recovery failed. Try again?")

            self.rate.sleep()

        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for motor in drive_motors:
            motor.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_odrive_interface_arm()
    rospy.spin()
