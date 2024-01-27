import odrive
from odrive.enums import *
from fibre import ObjectLostError
import time
from odrive.utils import dump_errors

print("finding an odrive...")
odrv_shoulder = odrive.find_any()


odrv_shoulder.config.brake_resistor0.resistance = 3 # resistance in Ohms
odrv_shoulder.config.brake_resistor0.enable = True
odrv_shoulder.clear_errors()



odrv_shoulder.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT

odrv_shoulder.axis0.config.motor.pole_pairs = 11

odrv_shoulder.axis0.config.motor.torque_constant = 8.27 / 136

odrv_shoulder.axis0.config.motor.calibration_current = 2.5

odrv_shoulder.axis0.config.motor.resistance_calib_max_voltage = 2

odrv_shoulder.axis0.config.calibration_lockin.current = 2.5


try:
    odrv_shoulder.axis0.requested_state = AxisState.MOTOR_CALIBRATION

except Exception as e:
     print(f"An error occurred: {e}")
else:
    odrv_shoulder.save_configuration()




odrv_shoulder.axis0.config.motor.current_soft_max = 5
odrv_shoulder.axis0.config.motor.current_hard_max = 5


odrv_shoulder.axis0.controller.config.vel_limit = 1



odrv_shoulder.inc_encoder0.config.cpr = 8192
odrv_shoulder.inc_encoder0.config.enabled = True
odrv_shoulder.axis0.config.load_encoder = EncoderId.INC_ENCODER0
odrv_shoulder.axis0.config.commutation_encoder = EncoderId.INC_ENCODER0

try:
    odrv_shoulder.save_configuration()# or save_configuration() or erase_configuration()
except ObjectLostError:
    pass # yeah, the object disappeared, I know

print("Reconnecting to ODrive")
odrv = odrive.find_any()

# [wait for ODrive to reboot]
odrv_shoulder.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
# [wait for motor to stop]
while (odrv.axis0.current_state != AxisState.IDLE):
    time.sleep(0.5)

odrv_shoulder.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL



#odrv_shoulder.axis0.controller.input_pos = 1
print("Sucess")

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
    odrv_shoulder.axis0.controller.input_pos = setpoint
    dump_errors(odrv_shoulder)
    time.sleep(0.01)