import odrive
from odrive.config import Axis
import odrive.enums
import odrive.utils
import odrive.config
from odrive.enums import AxisState, ProcedureResult, AxisError
from odrive.utils import dump_errors

import ODrive_utils_arm
import time
import math
import threading
# https://discourse.odriverobotics.com/t/odrive-s1-configuration-and-calibration-via-python/10455/6

# Min/Max phase inductance of motor
MIN_PHASE_INDUCTANCE = 0
MAX_PHASE_INDUCTANCE = 0.002

# Min/Max phase resistance of motor
MIN_PHASE_RESISTANCE = 0
MAX_PHASE_RESISTANCE = 1

# Tolerance for encoder offset float
ENCODER_OFFSET_FLOAT_TOLERANCE = 0.5

def __init__(self, axis_num: int):
    """
    Initialize RBEMotorConfig class by finding odrive and grabbing specified
    axis object.

    :param axis_num: Which channel/motor on the odrive your referring to.
    :type axis_num: int (0 or 1)
    """

    self.axis_num = axis_num

    # Connect to Odrive
    print("Looking for ODrive...")
    self._find_odrive()
    print("  Found ODrive.")
    print("    Bus Volage: " + str(self.odrv.vbus_voltage))

def _find_odrive(self):
    # connect to Odrive
    self.odrv = odrive.find_any()
    self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))

def erase_config(self):
    """ Erase pre-existing configuration """

    print("Erasing pre-existing configuration...")
    try:
        self.odrv.erase_configuration()
    except fibre.libfibre.ObjectLostError:
        # erasing configuration makes the ODrive reboot
        pass

def save_config(self):
    print("Saving configuration...")
    try:
        self.odrv.save_configuration()
        while self.odrv_axis.procedure_result != ProcedureResult.SUCCESS:
            time.sleep(0.5)
    except fibre.libfibre.ObjectLostError:
        # saving configuration makes the ODrive reboot
        print(" lost connection ...")
        self._find_odrive()
        if (self.odrv):
            print("  re-connected")
        pass

def show_errors(self):
    dump_errors(self.odrv)
    if self.odrv.axis0.active_errors > 0:
        print("  active error; clearing errors")
        self.odrv.clear_errors()

def dump_state(self):
    print("axis0 atate")
    print("  Current state:" + str(self.odrv.axis0.current_state))
    print("  Requested state:" + str(self.odrv.axis0.requested_state))


def set_odrive_parameters(self):
    print("set odrive parameters")
    self._find_odrive()

    # Configure Power Supply

    # Voltage limits
    batt_n_cells = 10
    self.odrv.config.dc_bus_undervoltage_trip_level = 24
    self.odrv.config.dc_bus_overvoltage_trip_level = 4.25 * batt_n_cells

    # Current Limits
    batt_ah = 4.4
    self.odrv.config.dc_max_positive_current = batt_ah * 2  # ~10A
    self.odrv.config.dc_max_negative_current = -2
    self.odrv.config.brake_resistor0.enable = True
    self.odrv.config.brake_resistor0.resistance = 50

    """ Saves odrive axis, motor, encoder and controller parameters """

    # BotWheel values
    self.odrv_axis.config.motor.motor_type = MOTOR_TYPE_HIGH_CURRENT
    self.odrv_axis.config.motor.pole_pairs = 15
    self.odrv_axis.config.motor.torque_constant = 8.27 / 8.7
    self.odrv_axis.config.motor.calibration_current = 4
    self.odrv_axis.config.calibration_lockin.current = 4
    self.odrv_axis.config.motor.resistance_calib_max_voltage = 10
    self.odrv_axis.config.motor.current_control_bandwidth = 100

    """It is recommended to set the soft max to something quite weak to start with,
       but strong enough to overcome friction in your system with a decent margin.
       Once you have built confidence in the stability of the control and strength 
       of your mechanical setup, you can increase these. For high current motors you
       need to turn this up to get high performance, and for low current motors such
       as the BotWheel you need to use something that’s lower than the examples shown above.

       The recommended maximum for current_soft_max is the continuous current rating 
       of your motor if you are not using motor thermistor temperature feedback, and 
       the peak current rating of your motor if you are using it. The hard max is a 
       fault trip level and should be set to a level above the soft max. The appropriate 
       level is a tradeoff between getting nuisance faults especially during high 
       accelerations, and ability to catch unstable current control situations. The 
       recommended maximum is the current your motor can handle for 50ms."""
    self.odrv_axis.config.motor.current_soft_max = 5
    self.odrv_axis.config.motor.current_hard_max = 9

    # 20kOhms
    self.odrv_axis.motor.motor_thermistor.config.r_ref = 20000
    self.odrv_axis.motor.motor_thermistor.config.beta = 3950
    # Thermistor must be connected
    self.odrv_axis.motor.motor_thermistor.config.temp_limit_lower = 80
    self.odrv_axis.motor.motor_thermistor.config.temp_limit_upper = 100
    self.odrv_axis.motor.motor_thermistor.config.enabled = True

    # System Configuration
    self.odrv_axis.config.torque_soft_min = -5
    self.odrv_axis.config.torque_soft_max = 5

    # [configure commutation encoder]
    self.odrv_axis.config.encoder_bandwidth = 100
    self.odrv.hall_encoder0.config.enabled = True
    self.odrv_axis.config.commutation_encoder = EncoderId.HALL_ENCODER0

    # [configure load encoder]
    self.odrv.inc_encoder0.config.cpr = 3200
    self.odrv.inc_encoder0.config.enabled = True
    self.odrv_axis.config.load_encoder = EncoderId.INC_ENCODER0

    # Tuned values
    self.odrv_axis.controller.config.pos_gain = 3
    self.odrv_axis.controller.config.vel_gain = 1.3
    self.odrv_axis.controller.config.vel_integrator_gain = 15
    self.odrv_axis.controller.config.vel_limit = 4
    self.odrv_axis.controller.config.vel_limit_tolerance = 2

    # Set to position control mode, so we can control the position of the
    # wheel
    self.odrv_axis.controller.config.input_mode = InputMode.PASSTHROUGH
    self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    # In the next step we are going to start powering the motor, so we
    # want to make sure that some of the above settings that require a
    # reboot are applied first.
    self.save_config()

def motor_calibration(self):
    self._find_odrive()

    print("Calibrating Odrive for RBE-202024-003 motor (you should hear a "
          "beep)...")

    self.odrv_axis.requested_state = AxisState.MOTOR_CALIBRATION
    #self.odrv_axis.motor_calibration()

    while self.odrv_axis.current_state != AXIS_STATE_IDLE:
        time.sleep(1)

    if self.odrv_axis.disarm_reason != 0:
        print("Disarmed: {}".format(self.odrv_axis.disarm_reason))
        print("Current state: {}".format(self.odrv_axis.current_state))
        self.show_errors()
        sys.exit(1)

    # Wait for calibration to take place
    time.sleep(20)

    if self.odrv_axis.active_errors != 0:
        print("Error: Odrive reported an error of {} while in the state "
              "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
              "debug:\n{}".format(self.odrv_axis.motor.error, self.odrv_axis.motor))

        sys.exit(1)

    self.save_config()

    """
    if self.odrv_axis.config.motor.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
            self.odrv_axis.config.motor.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
        print("Error: After odrive motor calibration, the phase inductance "
              "is at {}, which is outside of the expected range. Either widen the "
              "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
              "is between {} and {} respectively) or debug/fix your setup. Printing "
              "out Odrive motor data for debug:\n{}".format(self.odrv_axis.config.motor.phase_inductance,
                                                            self.MIN_PHASE_INDUCTANCE,
                                                            self.MAX_PHASE_INDUCTANCE,
                                                            self.odrv_axis.motor))

        sys.exit(1)

    if self.odrv_axis.config.motor.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
            self.odrv_axis.config.motor.phase_resistance >= self.MAX_PHASE_RESISTANCE:
        print("Error: After odrive motor calibration, the phase resistance "
              "is at {}, which is outside of the expected range. Either raise the "
              "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
              "debug/fix your setup. Printing out Odrive motor data for "
              "debug:\n{}".format(self.odrv_axis.config.motor.phase_resistance,
                                  self.MIN_PHASE_RESISTANCE,
                                  self.MAX_PHASE_RESISTANCE,
                                  self.odrv_axis.motor))

        sys.exit(1)

    # If all looks good, then lets tell ODrive that saving this calibration
    # to persistent memory is OK
    self.odrv_axis.motor.config.pre_calibrated = True
    
    """

def encoder_calibration(self):
    print("encoder calibration")
    self._find_odrive()

    # [CALIBRATE COMMUTATION ENCODER]
    print("  Calibrate commutation encoder")
    self.odrv.axis0.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION
    # [wait for motor to stop]
    time.sleep(10)

    self.odrv.axis0.requested_state = AxisState.ENCODER_HALL_PHASE_CALIBRATION
    # [wait for motor to stop]
    time.sleep(10)

    # [CALIBRATE LOAD ENCODER]
    print("  Calibrate load encoder")
    self.odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    # [wait for motor to stop]
    time.sleep(10)

    self.save_config()

    # If all looks good, then lets tell ODrive that saving this calibration
    # to persistent memory is OK
    #self.odrv.axis0.encoder.config.pre_calibrated = True

    self.show_errors()

def configure(self):
    """
    Configures the odrive device for RBE-1020-24-003 motor.
    """
    self.set_odrive_parameters()

    self.show_errors()
    print("Odrive configuration finished.")

def full_calibration(self):
    """
    Make full calibration (MOTOR + ENCODER) of the motor.
    """

    print("Full calibration ")
    self.odrv_axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    # Wait for calibration to take place
    time.sleep(40)

def mode_idle(self):
    """
    Puts the motor in idle (i.e. can move freely).
    """

    self.odrv_axis.requested_state = AXIS_STATE_IDLE

def mode_close_loop_control(self):
    """
    Puts the motor in closed loop control.
    """

    self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def move_input_pos(self, new_angle):
    """
    Puts the motor at a certain angle.

    :param new_angle:
    :param new_angle: Angle you want the motor to move.
    :type new_angle: int or float
    """

    self.odrv_axis.controller.input_pos = new_angle / 360.0