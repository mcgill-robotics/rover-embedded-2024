# from odrive_interface.msg import MotorError, MotorState
# import odrive
# from odrive.enums import AxisState, ProcedureResult
# odrive_serial = "386434413539"
# found_motor = odrive.find_any(serial_number=odrive_serial, timeout=5)

from __future__ import print_function

import odrive
from odrive.enums import AxisState, ProcedureResult
from odrive.utils import dump_errors
import time
import math
import threading


def watchdog():
    while not watchdog_stop_event.is_set():
        print(
            "Current state: "
            + str(odrv_shoulder.axis0.current_state)
            + ", "
            + "Raw angle: "
            + str(odrv_shoulder.rs485_encoder_group0.raw)
            + ", "
            + "input_pos="
            + str(odrv_shoulder.axis0.controller.input_pos)
            + ", "
            + "closed_loop_index="
            + str(int(AxisState.CLOSED_LOOP_CONTROL))
        )
        time.sleep(
            0.2
        )  # Set the period for the watchdog prints (0.2 seconds in this example)


# Find connected ODrive
print("finding an odrive...")
odrv_shoulder = odrive.find_any()

# Start the watchdog thread
watchdog_stop_event = threading.Event()
watchdog_thread = threading.Thread(target=watchdog)
watchdog_thread.start()

# SHOULDER CONFIG -------------------------------------------------------------------------
# odrv_shoulder.config.dc_bus_overvoltage_trip_level = 30
# odrv_shoulder.config.dc_bus_undervoltage_trip_level = 10.5
# odrv_shoulder.config.dc_max_positive_current = 10
# odrv_shoulder.config.brake_resistor0.enable = True
# odrv_shoulder.config.brake_resistor0.resistance = 2
# odrv_shoulder.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
# odrv_shoulder.axis0.config.motor.torque_constant = 0.06080882352941176
# odrv_shoulder.axis0.config.motor.pole_pairs = 11
# odrv_shoulder.axis0.config.motor.current_soft_max = 10
# odrv_shoulder.axis0.config.motor.current_hard_max = 23
# odrv_shoulder.axis0.config.motor.calibration_current = 2.5
# odrv_shoulder.axis0.config.motor.resistance_calib_max_voltage = 2
# odrv_shoulder.axis0.config.calibration_lockin.current = 2.5
# odrv_shoulder.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
# odrv_shoulder.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
# odrv_shoulder.axis0.config.torque_soft_min = -0.12161764705882352
# odrv_shoulder.axis0.config.torque_soft_max = 0.12161764705882352
# odrv_shoulder.can.config.protocol = Protocol.NONE
# odrv_shoulder.config.enable_uart_a = False
# odrv_shoulder.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_POLLING
# odrv_shoulder.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
# odrv_shoulder.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

# Calibration and save config -------------------------------------------------------------------------
print("starting calibration...")
odrv_shoulder.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
odrv_shoulder.axis0.motor.config.pre_calibrated = True
odrv_shoulder.axis0.config.startup_encoder_offset_calibration = True
odrv_shoulder.axis0.config.startup_closed_loop_control = True
odrv_shoulder.save_configuration()
# odrv_shoulder.reboot()

while odrv_shoulder.axis0.current_state != AxisState.IDLE:
    time.sleep(0.1)

odrv_shoulder.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

# Set reference -------------------------------------------------------------------------
odrv_shoulder.axis0.pos_vel_mapper.config.offset = 0.244
odrv_shoulder.axis0.pos_vel_mapper.config.offset_valid = True
odrv_shoulder.axis0.pos_vel_mapper.config.approx_init_pos = 0.244
odrv_shoulder.axis0.pos_vel_mapper.config.approx_init_pos_valid = True
odrv_shoulder.axis0.controller.config.absolute_setpoints = True


# Status -------------------------------------------------------------------------
print("Serial is {}".format(odrv_shoulder.serial_number))
print("Bus voltage is " + str(odrv_shoulder.vbus_voltage) + "V")
print("Position setpoint is " + str(odrv_shoulder.axis0.controller.pos_setpoint))
print("Requested state is " + str(odrv_shoulder.axis0.requested_state))
print("Current state is " + str(odrv_shoulder.axis0.current_state))
print("Current state is " + str(odrv_shoulder.rs485_encoder_group0.config.mode))

# And this is how function calls are done:
for i in [1, 2, 3, 4]:
    print("voltage on GPIO{} is {} Volt".format(i, odrv_shoulder.get_adc_voltage(i)))

while True:
    try:
        setpoint = float(
            input("Enter setpoint: ")
        )  # Convert the input to float for position control
    except ValueError:
        print("Invalid input. Please enter a numeric value.")
        continue  # Skip the rest of the loop and prompt for input again

    print(
        f"goto {int(setpoint)}, currently at {odrv_shoulder.rs485_encoder_group0.raw}, state {odrv_shoulder.axis0.current_state}"
    )
    odrv_shoulder.axis0.controller.input_pos = setpoint
    dump_errors(odrv_shoulder)
    time.sleep(0.01)

# Stop watchdog thread, when closing the script -------------------------------------------------
watchdog_stop_event.set()
watchdog_thread.join()
