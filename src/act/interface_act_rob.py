from abc import abstractmethod
from collections.abc import Iterable

from interface_act import RobotOut

import sys, tty, termios
import threading

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

from dynamixel_sdk import *    # Uses Dynamixel SDK library
from peripherals_def.dynamixel_communication.dynamixel_def import *     # Constant definitions

import time
import numpy as np

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class RealRobot(RobotOut):
    def __init__(self,
                motor_list,
                limits_file='/locomotion/trajectories/limits.txt', 
                time_step=None):
        super(RealRobot, self).__init__(time_step)

        self.motor_lock = threading.Lock()
        
        self.motor_ids = range(1,len(motor_list)+1)
        #Data structures defined: 
        #   portHandler, 
        self.portHandler = PortHandler(DEVICENAME) # Initialize port handler
        #   packetHandler, 
        self.packetHandler = PacketHandler(PROTOCOL_VERSION) # Initialize packet handler
        #   groupSyncRead, current position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)       
        #   groupSyncWrite, profile velocity and goal position
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_VEL_AND_POS)

        #Open port, Set baudrate, Enable Torque of all motors, add param storage for SyncRead

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        
        #Setup motors.
        self.motor_lock = threading.Lock()   
        #Store min, max and zero joint positions from the file.
        self.motor_zeros = np.zeros(len(self.motor_ids))
        minpos = np.zeros(len(self.motor_ids), dtype='int')
        maxpos = np.zeros(len(self.motor_ids), dtype='int')
        with open(limits_file,"r") as f:
                counter = 0
                for line in f:
                    fields = line.split(";")
                    self.motor_zeros[counter] = fields[0]
                    minpos[counter] = fields[1]
                    maxpos[counter] = fields[2]
                    counter = counter + 1

        self.set_pos_limits(self.motor_ids, minpos, maxpos)   

        self.minpos, self.maxpos = self.get_pos_limits(self.motor_ids)

        self.torque_enable(self.motor_ids)

        # Add parameter storage for current position value
        for i in self.motor_ids:
            dxl_addparam_result = self.groupSyncRead.addParam(i)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % i)
                quit()

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

    def set_pos_limits(self, motor_id, minPosLim, maxPosLim):
        """
        Set pos limits for motor with ids
        posLim registers are in the EEPROM, so the code first disables torque, 
        then applies the change and finally enables the torque again
        Note that if goal position is outside of the limits the motor will not move and you will get a dxl_error
        """
        if not(isinstance(motor_id, Iterable)): 
            # se non è iterable lo si rende iterable
            motor_id = [motor_id] 
        
        self.torque_disable(motor_id)
        for i in range(len(motor_id)):
            # set minPosLim
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id[i], ADDR_MIN_POS_LIM, minPosLim[i])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # set maxPosLim
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id[i], ADDR_MAX_POS_LIM, maxPosLim[i])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.minpos, self.maxpos = self.get_pos_limits(self.motor_ids) #update minpos and maxpos variables
        
        self.torque_enable(motor_id)


    def get_pos_limits(self, motor_id):
        """
        pos limits are output in register values in arrays dxl_min_pos dxl_max_pos. 
        Later they are converted in radians using method:q_from_d
        """
        if not(isinstance(motor_id, Iterable)): 
            # se non è iterable lo si rende iterable
            motor_id = [motor_id] 
        dxl_min_pos = np.zeros(len(motor_id),dtype = 'int')#[0 for i in ids]
        dxl_max_pos = np.zeros(len(motor_id),dtype = 'int')#[0 for i in ids]
        for i in range(len(motor_id)):
            # get minPosLim
            dxl_min_pos[i], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id[i], ADDR_MIN_POS_LIM)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # get maxPosLim
            dxl_max_pos[i], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id[i], ADDR_MAX_POS_LIM)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        q_min_pos = self.q_from_d(motor_id, dxl_min_pos)
        q_max_pos = self.q_from_d(motor_id, dxl_max_pos)
        
        return q_min_pos, q_max_pos
    
    def d_from_q(self, motor_id, q):
        """      
        Input motor_id and joint coordinate in radians (q)
        Outputs d the goal position command of dynamixel
        it requires the motor_zeros calibrated (or passed) in the contructor
        """
        if not(isinstance(motor_id, Iterable)): 
            # se non è iterable lo si rende iterable
            motor_id = [motor_id] 
            q = [q]
        
        d = np.zeros(len(motor_id), dtype = 'int')
        for i in range(len(motor_id)):
                d[i] = self.sign[motor_id[i]-1]*q[i]/POS_UNIT + self.motor_zeros[motor_id[i]-1]
        return d

    def q_from_d(self, motor_id, d):
        """      
        Input motor_id and motor position in register value (d) 
        Outputs q the corresponding value in radians
        it requires the motor_zeros calibrated (or passed) in the contructor
        PAY ATTENTION: POS_UNIT must convert from register value to radians
        """
        if not(isinstance(motor_id, Iterable)): 
            # se non è iterable lo si rende iterable
            motor_id = [motor_id] 
            d = [d]
        q = np.zeros(len(motor_id))
        for i in range(len(motor_id)):
                q[i] = self.sign[motor_id[i]-1]*(d[i] - self.motor_zeros[motor_id[i]-1])*POS_UNIT
        return q      

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
