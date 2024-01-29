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
    "ARM_0": "62003024573753",
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
                + str(AxisState(odrv.axis0.current_state).name)
                + ", "
                + "Raw angle: "
                + str(odrv.rs485_encoder_group0.raw)
                + ", "
                + "Current pos: "
                + str(odrv.axis0.pos_vel_mapper.pos_rel)
                + ", "
                + "input_pos="
                + str(odrv.axis0.controller.input_pos)
                # + ", velocity"
                # + str(odrv_shoulder.axis0.controller.velocity)
            )
            time.sleep(
                1
            )  # Set the period for the watchdog prints (0.2 seconds in this example)
        except NameError:
            pass


# Find connected ODrive
print("finding an odrive...")
odrv_shoulder = odrive.find_any()


# SHOULDER CONFIG -------------------------------------------------------------------------
odrv = odrv_shoulder

# ERASE CONFIG
try:
    odrv.erase_configuration()
except fibre.libfibre.ObjectLostError:
    # erasing configuration makes the ODrive reboot
    pass

odrv_shoulder = odrive.find_any()
odrv = odrv_shoulder

odrv.config.dc_bus_overvoltage_trip_level = 30
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = 10
odrv.config.brake_resistor0.enable = True
odrv.config.brake_resistor0.resistance = 2
odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv.axis0.config.motor.torque_constant = 0.06080882352941176
odrv.axis0.config.motor.pole_pairs = 11
odrv.axis0.config.motor.current_soft_max = 10
odrv.axis0.config.motor.current_hard_max = 23
odrv.axis0.config.motor.calibration_current = 2.5
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 2.5
odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv.axis0.config.torque_soft_min = -0.12161764705882352
odrv.axis0.config.torque_soft_max = 0.12161764705882352
odrv.can.config.protocol = Protocol.NONE
odrv.config.enable_uart_a = False
odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_POLLING
odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

# SAVE CONFIG
try:
    odrv.save_configuration()
    while odrv.axis0.procedure_result != ProcedureResult.SUCCESS:
        time.sleep(0.5)
except fibre.libfibre.ObjectLostError:
    # saving configuration makes the ODrive reboot
    print(" lost connection ...")
    odrv = odrive.find_any()
    if odrv:
        print("  re-connected")
    pass

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

# MANUAL
odrv.clear_errors()
if odrv.axis0.current_state != AxisState.FULL_CALIBRATION_SEQUENCE:
    odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
time.sleep(0.2)

# Wait for calibration to end
while (
    # odrv.axis0.current_state == AxisState.MOTOR_CALIBRATION
    # or odrv.axis0.current_state == AxisState.FULL_CALIBRATION_SEQUENCE
    not odrv.axis0.current_state
    == AxisState.IDLE
):
    time.sleep(1)
    print(
        "Motor {} is still calibrating. Current state: {}".format(
            odrv.serial_number, AxisState(odrv.axis0.current_state).name
        )
    )
results_available = False

# Wait for calibration results
while not results_available:
    results_available = True
    if odrv.axis0.procedure_result == ProcedureResult.BUSY:
        results_available = False
    time.sleep(0.1)

# ERROR CHECKING
calibration_failed = False
result = odrv.axis0.procedure_result
errors = odrv.axis0.active_errors
if result != ProcedureResult.SUCCESS:
    calibration_failed = True
    if errors != 0:
        print(
            "Motor error(s) in motor {}: {}".format(
                odrv.serial_number, AxisError(odrv.axis0.disarm_reason).name
            )
        )
    print(
        "Calibration procedure failed in motor {}. Reason: {}".format(
            odrv.serial_number, ProcedureResult(result).name
        )
    )
    if result == ProcedureResult.DISARMED:
        print(
            "Motor {} disarmed. Reason: {}".format(
                odrv.serial_number, AxisError(odrv.axis0.disarm_reason).name
            )
        )
    odrv.axis0.requested_state = AxisState.IDLE
else:
    print("Calibration successful!")

# ENTER CLOSED LOOP CONTROL
odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
while odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
    time.sleep(1)
    print(
        "Motor {} is still entering closed loop control. Current state: {}".format(
            odrv.serial_number, AxisState(odrv.axis0.current_state).name
        )
    )

# TODO - find a way to skip calibration if already calibrated
# odrv.axis0.config.motor.pre_calibrated = True

odrv.axis0.config.startup_encoder_offset_calibration = True
odrv.axis0.config.startup_closed_loop_control = True

# And this is how function calls are done:
def print_gpio_voltages():
    for i in [1, 2, 3, 4]:
        print("voltage on GPIO{} is {} Volt".format(i, odrv.get_adc_voltage(i)))

while True:
    try:
        setpoint = float(
            input("Enter setpoint: ")
        )  # Convert the input to float for position control
    except ValueError:
        print("Invalid input. Please enter a numeric value.")
        continue  # Skip the rest of the loop and prompt for input again

    print(
        f"goto {int(setpoint)}, currently at {odrv.rs485_encoder_group0.raw}, state {odrv.axis0.current_state}"
    )
    odrv.axis0.controller.input_pos = setpoint
    dump_errors(odrv)
    time.sleep(0.01)

# Stop watchdog thread, when closing the script -------------------------------------------------
watchdog_stop_event.set()
watchdog_thread.join()