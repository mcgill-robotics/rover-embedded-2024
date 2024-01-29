# from odrive_interface.msg import MotorError, MotorState
# import odrive
# from odrive.enums import AxisState, ProcedureResult
# odrive_serial = "386434413539"
# found_motor = odrive.find_any(serial_number=odrive_serial, timeout=5)

from __future__ import print_function

import odrive
from odrive.config import Axis
import odrive.enums
import odrive.utils
import odrive.config

# from odrive.enums import AxisState, ProcedureResult, AxisError, ControlMode, EncoderId, InputMode, MotorType
from odrive.enums import *
from odrive.utils import dump_errors

import ODrive_utils_arm
import time
import math
import threading
import fibre

# TODO find more
arm_serial_numbers = {
    "rover_arm_shoulder": "386434413539",
    "rover_arm_elbow": "0",
    "rover_arm_waist": "0",
}


def move_clockwise(in_pos):
    # odrv_shoulder.axis0.controller.config.
    odrv_shoulder.axis0.pos_vel_mapper.pos_rel
    odrv_shoulder.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    current_pos = odrv_shoulder.axis0.rs485_encoder_group0.raw * 8192
    odrv_shoulder.axis0.controller.input_pos = 2
    return


def watchdog():
    i = 0

    while not watchdog_stop_event.is_set():
        # i += 1
        # if (odrv_shoulder.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL or odrv_shoulder.axis0.current_state != AxisState.MOTOR_CALIBRATION or odrv_shoulder.axis0.current_state != AxisState.ENCODER_OFFSET_CALIBRATION):
        #     odrv_shoulder.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

        try:
            print(
                "Current state: "
                + str(AxisState(odrv_shoulder.odrv.axis0.current_state).name)
                + ", "
                + "Raw angle: "
                + str(odrv_shoulder.odrv.rs485_encoder_group0.raw)
                + ", "
                + "Current pos: "
                + str(odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_rel)
                + ", "
                + "input_pos="
                + str(odrv_shoulder.odrv.axis0.controller.input_pos)
                # + ", velocity"
                # + str(odrv_shoulder.axis0.controller.velocity)
            )
            time.sleep(
                1
            )  # Set the period for the watchdog prints (0.2 seconds in this example)
        except NameError:
            pass


class ODrive_Joint:
    def __init__(self, odrv):
        self.odrv = odrv
        # odrv.serial_number is int, serial_number is str of hex version
        self.serial_number = str(hex(self.odrv.serial_number)[2:])
        self.timeout = 5

    def save_config(self):
        try:
            self.odrv.save_configuration()
            while self.odrv.axis0.procedure_result != ProcedureResult.SUCCESS:
                time.sleep(0.5)
        # Saving configuration makes the ODrive reboot
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in save_config() ...")
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                print("  re-connected in save_config()")
            pass

    def erase_config(self):
        try:
            self.odrv.erase_configuration()
        # Erasing configuration makes the ODrive reboot
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in erase_config() ...")
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                print("  re-connected in erase_config()")
            pass

    def calibrate(self):
        self.odrv.clear_errors()
        if self.odrv.axis0.current_state != AxisState.FULL_CALIBRATION_SEQUENCE:
            self.odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
        time.sleep(0.2)

        # Wait for calibration to end
        while (
            # self.odrv.axis0.current_state == AxisState.MOTOR_CALIBRATION
            # or self.odrv.axis0.current_state == AxisState.FULL_CALIBRATION_SEQUENCE
            not self.odrv.axis0.current_state
            == AxisState.IDLE
        ):
            time.sleep(1)
            print(
                "Motor {} is still calibrating. Current state: {}".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
        results_available = False

        # Wait for calibration results
        while not results_available:
            results_available = True
            if self.odrv.axis0.procedure_result == ProcedureResult.BUSY:
                results_available = False
            time.sleep(0.1)

        # ERROR CHECKING
        calibration_failed = False
        result = self.odrv.axis0.procedure_result
        errors = self.odrv.axis0.active_errors
        if result != ProcedureResult.SUCCESS:
            calibration_failed = True
            if errors != 0:
                print(
                    "Motor error(s) in motor {}: {}".format(
                        self.odrv.serial_number,
                        AxisError(self.odrv.axis0.disarm_reason).name,
                    )
                )
            print(
                "Calibration procedure failed in motor {}. Reason: {}".format(
                    self.odrv.serial_number, ProcedureResult(result).name
                )
            )
            if result == ProcedureResult.DISARMED:
                print(
                    "Motor {} disarmed. Reason: {}".format(
                        self.odrv.serial_number,
                        AxisError(self.odrv.axis0.disarm_reason).name,
                    )
                )
            self.odrv.axis0.requested_state = AxisState.IDLE
        else:
            print("Calibration successful!")

        # ENTER CLOSED LOOP CONTROL
        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        while self.odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
            time.sleep(1)
            print(
                "Motor {} is still entering closed loop control. Current state: {}".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
        print(
            "SUCCESS: Motor {} is in closed loop control.".format(
                self.odrv.serial_number
            )
        )

        def print_gpio_voltages(self):
            for i in [1, 2, 3, 4]:
                print(
                    "voltage on GPIO{} is {} Volt".format(
                        i, self.odrv.get_adc_voltage(i)
                    )
                )


# Find connected ODrive
print("Finding ODrive...")
# odrv_shoulder = odrive.find_any()
test = arm_serial_numbers["rover_arm_shoulder"]
odrv_shoulder = ODrive_Joint(
    odrive.find_any(serial_number=arm_serial_numbers["rover_arm_shoulder"], timeout=5)
)

# ERASE CONFIG -----------------------------------------------------------------------
odrv_shoulder.erase_config()


odrv_shoulder.odrv.config.dc_bus_overvoltage_trip_level = 30
odrv_shoulder.odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv_shoulder.odrv.config.dc_max_positive_current = 10
odrv_shoulder.odrv.config.brake_resistor0.enable = True
odrv_shoulder.odrv.config.brake_resistor0.resistance = 2
odrv_shoulder.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv_shoulder.odrv.axis0.config.motor.torque_constant = 0.06080882352941176
odrv_shoulder.odrv.axis0.config.motor.pole_pairs = 11
odrv_shoulder.odrv.axis0.config.motor.current_soft_max = 10
odrv_shoulder.odrv.axis0.config.motor.current_hard_max = 23
odrv_shoulder.odrv.axis0.config.motor.calibration_current = 2.5
odrv_shoulder.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv_shoulder.odrv.axis0.config.calibration_lockin.current = 2.5
odrv_shoulder.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv_shoulder.odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv_shoulder.odrv.axis0.config.torque_soft_min = -0.12161764705882352
odrv_shoulder.odrv.axis0.config.torque_soft_max = 0.12161764705882352
odrv_shoulder.odrv.can.config.protocol = Protocol.NONE
odrv_shoulder.odrv.config.enable_uart_a = False
odrv_shoulder.odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_POLLING
odrv_shoulder.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
odrv_shoulder.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

odrv_shoulder.save_config()

# Start the watchdog thread for DEBUG INFO ---------------------------------------------------------
watchdog_stop_event = threading.Event()
watchdog_thread = threading.Thread(target=watchdog)
watchdog_thread.start()

# Calibration and save config -------------------------------------------------------------------------
print("starting calibration...")

# USING EREN'S CODE
# odrv_lst = [odrv_shoulder]
# is_calibrated = False
# while not is_calibrated:
#     ODrive_utils_arm.calibrate_motors(odrv_lst)  # us
#     print(
#     "Current state: "
#     + str(AxisState(odrv_shoulder.axis0.current_state).name))
#     dump_errors(odrv)

odrv_shoulder.calibrate()

# TODO - find a way to skip calibration if already calibrated
# odrv.axis0.config.motor.pre_calibrated = True

# TODO what is this for??
# odrv.axis0.config.startup_encoder_offset_calibration = True
# odrv.axis0.config.startup_closed_loop_control = True


while True:
    try:
        setpoint = float(
            input("Enter setpoint: ")
        )  # Convert the input to float for position control
    except ValueError:
        print("Invalid input. Please enter a numeric value.")
        continue  # Skip the rest of the loop and prompt for input again

    print(
        f"goto {int(setpoint)}, currently at {odrv_shoulder.odrv.rs485_encoder_group0.raw}, state {odrv_shoulder.odrv.axis0.current_state}"
    )
    odrv_shoulder.odrv.axis0.controller.input_pos = setpoint
    dump_errors(odrv_shoulder.odrv)
    time.sleep(0.01)

# Stop watchdog thread, when closing the script -------------------------------------------------
watchdog_stop_event.set()
watchdog_thread.join()
