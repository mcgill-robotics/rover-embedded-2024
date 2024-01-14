import odrive
from odrive.enums import *

odrive_serial_number = "....."  # Replace with your ODrive's serial number


odrive_board = odrive.find_any(serial_number=odrive_serial_number)
#Now we have the odrive_board object which has methods



