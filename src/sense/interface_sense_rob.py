from abc import abstractmethod

from interface_sense import RobotSensorsDrive

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
import time
import numpy as np

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class RealRobot(RobotSensorsDrive):
    def __init__(self,
                motor_list,
                 time_step=None):
        super(RealRobot, self).__init__(time_step)
        

    def torque_enable(self, motor_id):
        if type(ids)==np.int32:
            ids = np.array([ids],dtype="int")
        for i in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d - Torque Enabled" % i)

    def torque_disable(self, motor_id):
        #Disable torque for relId motors. relId is an array with 1,2 or 3 elements
        # handles scalar case
        if type(ids)==np.int32:
            ids = np.array([ids],dtype="int")
        for i in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d - Torque Disabled" % i)  
    
    @abstractmethod
    def get_pos(self,motor_id):
        """
        Return the current position of a given motor, in radians
        """
        pass

    def set_pos_limits(self, motor_id):
        # Set pos limits for motor with ids
        # posLim registers are in the EEPROM, so the code first disable torque, the applies the change and finally enable the torque again
        # Note that if goal position is outside of the limits the motor will not move and you will get a dxl_error
        self.torque_disable(motor_id)

        counter = 0
        for i in motor_id:
            # set minPosLim
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_MIN_POS_LIM, minPosLim[counter])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # set maxPosLim
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_MAX_POS_LIM, maxPosLim[counter])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            counter = counter + 1

        self.minpos, self.maxpos = self.get_pos_lim(self.motor_ids) #update minpos and maxpos variables
        self.torque_enable(motor_id)


    def get_pos_limits(self, motor_id):
        # Get pos limits for motor with ids
        dxl_min_pos = np.zeros(len(motor_id),dtype = 'int')#[0 for i in ids]
        dxl_max_pos = np.zeros(len(motor_id),dtype = 'int')#[0 for i in ids]
        counter = 0
        for i in ids:
            # get minPosLim
            dxl_min_pos[counter], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, i, ADDR_MIN_POS_LIM)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # get maxPosLim
            dxl_max_pos[counter], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, i, ADDR_MAX_POS_LIM)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            counter = counter + 1

        return dxl_min_pos, dxl_max_pos

    @abstractmethod
    def goto_pos(self, motor_id, vel, pos):
        """
        command a position to the motor identified by motor_id to be reached and a velocity to reach it
        NOTA: differenza tra simualtore e motori
        """
        pass


    def get_info(self, motor_id):
        #returns status for all motors with relId
        #if dxl_status[i] is 32, motor i is in overload
        dxl_status = np.zeros(len(ids),dtype = 'int')#[0 for i in ids]
        counter = 0
        for i in motor_id:
            # get status
            dxl_status[counter], dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, i, ADDR_ERROR_STATUS)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            counter = counter + 1
        return dxl_status

    def reboot(self, motor_id):
        # Applies the  reboot procedure 
        counter = 0
        for i in motor_id:
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, i)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            counter = counter + 1