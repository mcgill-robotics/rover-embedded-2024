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
from msg import MotorState, MotorError


# TODO find more serial, it is a string of hex of the serial number
arm_serial_numbers = {
    "rover_arm_shoulder": "386434413539",  # 0x386434413539 = 62003024573753 in decimal
    "rover_arm_elbow": "0",
    "rover_arm_waist": "0",
}

arm_joint_lst = []


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
        self.outshaft_subscriber = rospy.Subscriber(
            "/arm_lower_output_shaft_position",
            Float32MultiArray,
            self.handle_arm_outshaft_fb,
        )
        self.command_subscriber = rospy.Subscriber(
            "/armBrushlessCmd", Float32MultiArray, self.handle_arm_command
        )

        # Publishers
        self.state_publisher = rospy.Publisher(
            "/odrive_arm_state", MotorState, queue_size=1
        )
        self.feedback_publisher = rospy.Publisher(
            "/armBrushlessFB",  # Change this to your desired topic name
            Float32MultiArray,
            queue_size=10,
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Update encoder angle from external encoders
    def handle_arm_outshaft_fb(self, msg):
        self.elbow_pos_outshaft = msg.data[0]
        self.shoulder_pos_outshaft = msg.data[1]
        self.waist_pos_outshaft = msg.data[2]
        self.feedback_publisher.publish(msg)

    # Receive setpoint from external control node
    def handle_arm_command(self, msg):
        self.elbow_pos_setpoint = msg.data[0]
        self.shoulder_pos_setpoint = msg.data[1]
        self.waist_pos_setpoint = msg.data[2]

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
            # TODO ODrive feedback
            # Get feedback and publish it to "/wheel_velocity_feedback"
            # feedback = WheelSpeed()
            # measured_speed_lb = -drive_lb.encoder_estimator0.vel_estimate
            # measured_speed_lf = -drive_lf.encoder_estimator0.vel_estimate
            # measured_speed_rb = drive_rb.encoder_estimator0.vel_estimate
            # measured_speed_rf = drive_rf.encoder_estimator0.vel_estimate

            # feedback.left[0], feedback.left[1] = measured_speed_lb, measured_speed_lf
            # feedback.right[0], feedback.right[1] = measured_speed_rb, measured_speed_rf
            # self.feedback_publisher.publish(feedback)

            # APPLY SETPOINT
            # TODO Elbow and waist motors
            shoulder_pos_error = self.shoulder_pos_setpoint - self.shoulder_pos_outshaft
            shoulder_pos_increment = shoulder_pos_error * odrv_shoulder.gear_ratio
            shoulder_pos_setpoint = (
                odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_rel + shoulder_pos_increment
            )
            odrv_shoulder.odrv.axis0.controller.input_pos = shoulder_pos_setpoint

            # PRINT POSITIONS TO CONSOLE
            print(
                f"\rElbow: {round(self.elbow_pos_outshaft, 2)}, Shoulder: {round(self.shoulder_pos_outshaft, 2)}, Waist: {round(self.waist_pos_outshaft, 2)}",
                end="",
            )

            # SEND ODRIVE INFO AND HANDLE ERRORS
            for joint in arm_joint_lst:
                state_fb = MotorState()
                state_fb.id = joint.serial_number
                state_fb.state = joint.odrv.axis0.current_state
                state_fb.pos_rel = joint.odrv.axis0.pos_vel_mapper.pos_rel
                state_fb.pos_abs = joint.odrv.axis0.pos_vel_mapper.pos_abs
                state_fb.input_pos = joint.odrv.axis0.controller.input_pos
                self.state_publisher.publish(state_fb)

                # ERROR HANDLING
                if joint.axis0.active_errors != 0:
                    # Tell the rover to stop
                    for joint in arm_joint_lst:
                        joint.odrv.axis0.controller.input_vel = 0

                    # Wait until it actually stops
                    motor_stopped = False
                    while not motor_stopped:
                        motor_stopped = True
                        for joint in arm_joint_lst:
                            if abs(joint.odrv.encoder_estimator0.vel_estimate) >= 0.01:
                                motor_stopped = False
                        if rospy.is_shutdown():
                            print(
                                "Shutdown prompt received. Setting all motors to idle state."
                            )
                            for joint in arm_joint_lst:
                                joint.odrv.axis0.requested_state = AxisState.IDLE
                            break

                    # Wait for two seconds while all the transient currents and voltages calm down
                    rospy.sleep(5)

                    # Now try to recover from the error. This will always succeed the first time, but if
                    # the error persists, the ODrive will not allow the transition to closed loop control, and
                    # re-throw the error.
                    joint.clear_errors()
                    rospy.sleep(0.5)
                    joint.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                    rospy.sleep(0.5)

                    # If the motor cannot recover successfully publish a message about the error, then print to console
                    if joint.axis0.active_errors != 0:
                        error_fb = MotorError()
                        error_fb.id = joint.serial_number
                        # error_fb.error = decode_errors(joint.axis0.active_errors)
                        error_fb.error = ODriveError(joint.axis0.active_errors).name
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
                                joint.clear_errors()
                                rospy.sleep(0.5)
                                joint.odrv.axis0.requested_state = (
                                    AxisState.CLOSED_LOOP_CONTROL
                                )
                                rospy.sleep(0.5)
                                if joint.odrv.axis0.active_errors == 0:
                                    break
                                else:
                                    print("Recovery failed. Try again?")

            self.rate.sleep()

        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for joint in arm_joint_lst:
            joint.odrv.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_odrive_interface_arm()
    rospy.spin()
