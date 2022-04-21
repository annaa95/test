# Author: Giacomo Picardi
# Control table address - Control table address is different in Dynamixel model
ADDR_MAX_POS_LIM            = 48
ADDR_MIN_POS_LIM            = 52
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_ERROR_STATUS           = 70
ADDR_PRO_PROFILE_VELOCITY   = 112
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_ERROR_STATUS            = 1
LEN_MAX_POS_LIM             = 4
LEN_MIN_POS_LIM             = 4
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_VEL_AND_POS         = 8

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 3000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
REBOOT                      = 0                 # Value for rebooting error status register
