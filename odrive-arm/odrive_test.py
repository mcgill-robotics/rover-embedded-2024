from __future__ import print_function

import odrive
from odrive.config import Axis
import odrive.enums
import odrive.utils
import odrive.config

from odrive.enums import *
from odrive.utils import dump_errors

import time
import threading
import fibre

# TODO find more serial, it is a string of hex of the serial number
arm_serial_numbers = {
    "rover_arm_shoulder": "386434413539",
    "rover_arm_elbow": "0",
    "rover_arm_waist": "0",
}

# Set to True if you want to reapply the config, False if you want to skip it
reapply_config = False

# True by default, set to False if you don't want to calibrate
do_calibration = True


def watchdog():
    while not watchdog_stop_event.is_set():
        try:
            print(
                "current_state="
                + str(AxisState(odrv_shoulder.odrv.axis0.current_state).name)
                + ", "
                + "Raw angle="
                + str(odrv_shoulder.odrv.rs485_encoder_group0.raw)
                + ", "
                + "pos_rel="
                + str(odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_rel)
                + ", "
                + "pos_abs="
                + str(odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_abs)
                + ", "
                + "input_pos="
                + str(odrv_shoulder.odrv.axis0.controller.input_pos)
                + ", "
                + "vel_estimate="
                + str(odrv_shoulder.odrv.encoder_estimator0.vel_estimate)
            )
            time.sleep(1)
        except NameError:
            pass
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in watchdog() ...")
            odrv_shoulder.reconnect()
            if odrv_shoulder.odrv:
                print("  re-connected in watchdog()")
            pass


class ODrive_Joint:
    def __init__(self, odrv):
        self.odrv = odrv
        # odrv.serial_number is int, serial_number should be hex version in string
        self.serial_number = str(hex(self.odrv.serial_number)[2:])
        self.timeout = 5
        # gear_ratio is input revolutions / output revolutions
        self.gear_ratio = 1

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

    def reconnect(self):
        try:
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in reconnect() ...")
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                print("  re-connected in reconnect()")
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

    def enter_closed_loop_control(self):
        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        while (
            self.odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL
            or self.odrv.axis0.current_state == AxisState.IDLE
        ):
            time.sleep(0.5)
            print(
                "Motor {} is still entering closed loop control. Current state: {}".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            dump_errors(self.odrv)
        print(
            "SUCCESS: Motor {} is in state: {}.".format(
                self.odrv.serial_number,
                AxisState(self.odrv.axis0.current_state).name,
            )
        )

    # Print the voltage on the GPIO pins
    def print_gpio_voltages(self):
        for i in [1, 2, 3, 4]:
            print(
                "voltage on GPIO{} is {} Volt".format(i, self.odrv.get_adc_voltage(i))
            )


# CONNECT TO ODRIVE ------------------------------------------------------------------
print("FINDING ODrive...")
odrv_shoulder = ODrive_Joint(
    odrive.find_any(serial_number=arm_serial_numbers["rover_arm_shoulder"], timeout=5)
)
odrv_shoulder.gear_ratio = 3

# ERASE CONFIG -----------------------------------------------------------------------
if reapply_config:
    print("ERASING CONFIG...")
    odrv_shoulder.erase_config()

# APPLY CONFIG -----------------------------------------------------------------------
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


# SAVE CONFIG -----------------------------------------------------------------------
if reapply_config:
    print("SAVING CONFIG...")
    odrv_shoulder.save_config()


# CALIBRATE -------------------------------------------------------------------------
if do_calibration:
    print("CALIBRATING...")
    odrv_shoulder.calibrate()

# ENTER CLOSED LOOP CONTROL ---------------------------------------------------------
print("ENTERING CLOSED LOOP CONTROL...")
odrv_shoulder.enter_closed_loop_control()

# SAVE CALIBRATION -----------------------------------------------------------------
if reapply_config:
    # odrv_shoulder.odrv.axis0.motor.config.pre_calibrated = True
    odrv_shoulder.odrv.axis0.config.startup_motor_calibration = True
    odrv_shoulder.odrv.axis0.config.startup_encoder_offset_calibration = True
    odrv_shoulder.odrv.axis0.config.startup_closed_loop_control = True
    odrv_shoulder.save_config()

# SET ABSOLUTE POSITION ----------------------------------------------------------------
odrv_shoulder.odrv.axis0.set_abs_pos(6.9)
# odrv_shoulder.odrv.encoder_estimator0.pos_estimate = 6.9

# START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
watchdog_stop_event = threading.Event()
watchdog_thread = threading.Thread(target=watchdog)
watchdog_thread.start()

# PROMPT FOR SETPOINT (INCREMENTAL) ----------------------------------------------------------------
while True:
    try:
        setpoint_increment = float(
            input("Enter setpoint_increment (rev): ")
        )  # Convert the input to float for position control
    except ValueError:
        print("Invalid input. Please enter a numeric value.")
        continue  # Skip the rest of the loop and prompt for input again
    setpoint = odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_rel + (
        setpoint_increment * odrv_shoulder.gear_ratio
    )
    print(
        f"goto {float(setpoint)}, currently at {odrv_shoulder.odrv.rs485_encoder_group0.raw}, state {odrv_shoulder.odrv.axis0.current_state}"
    )
    print(
        f"increment {setpoint_increment}, currently at {odrv_shoulder.odrv.rs485_encoder_group0.raw}, state {odrv_shoulder.odrv.axis0.current_state}"
    )
    odrv_shoulder.odrv.axis0.controller.input_pos = setpoint
    dump_errors(odrv_shoulder.odrv)
    time.sleep(0.01)

# PROMPT FOR SETPOINT ----------------------------------------------------------------
# while True:
#     try:
#         setpoint = float(
#             input("Enter setpoint: ")
#         )  # Convert the input to float for position control
#     except ValueError:
#         print("Invalid input. Please enter a numeric value.")
#         continue  # Skip the rest of the loop and prompt for input again

#     print(
#         f"goto {int(setpoint)}, currently at {odrv_shoulder.odrv.rs485_encoder_group0.raw}, state {odrv_shoulder.odrv.axis0.current_state}"
#     )
#     odrv_shoulder.odrv.axis0.controller.input_pos = setpoint
#     dump_errors(odrv_shoulder.odrv)
#     time.sleep(0.01)


# Stop watchdog thread, when closing the script -------------------------------------------------
watchdog_stop_event.set()
watchdog_thread.join()
