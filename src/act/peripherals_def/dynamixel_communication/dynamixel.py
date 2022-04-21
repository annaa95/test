#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Giacomo Picardi
# Class  Dynamixel abstracts serial comm for Dyn Servos

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from .dynamixel_def import *                          # Constant definitions
import time
import numpy as np

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class Dynamixel:
    def __init__(self,motor_ids):
        #Constructor
        #Data structures defined: portHandler, packetHandler, groupSyncRead, groupSyncWrite (some of them might be moved to class Exapod which is an array of Legs)
        #Open port, Set baudrate, Enable Torque of all motors, add param storage for SyncRead
        self.motor_ids = motor_ids
        self.portHandler = PortHandler(DEVICENAME) # Initialize port handler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION) # Initialize packet handler
        # Initialize GroupSyncWrite instance for profile velocity and goal position
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_PRO_PROFILE_VELOCITY, LEN_PRO_VEL_AND_POS)
        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

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

        self.minpos, self.maxpos = self.get_pos_lim(self.motor_ids)

        self.torque_enable(self.motor_ids)

        # Add parameter storage for present position value
        for i in self.motor_ids:
            dxl_addparam_result = self.groupSyncRead.addParam(i)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % i)
                quit()

    def __del__(self):
        #Destructor: clear syncread parameter storage, disable torque and close port
        # Clear syncread parameter storage
        self.groupSyncRead.clearParam()

        self.torque_disable(self.motor_ids)

        # Close port
        self.portHandler.closePort()

    def allocate_conf(self, ids, vel, pos):
        #Allocate velocity and position for all motors with relative Id specified in relId
        counter = 0
        # handles scalar case
        if type(ids)==np.int32:
            ids = np.array([ids],dtype="int")
            vel = np.array([vel],dtype="int")
            pos = np.array([pos],dtype="int")
        #if goal position command is outside of the limits, go to the limit
        for i in ids:
            if pos[counter] > self.maxpos[i-1]:
                pos[counter] = self.maxpos[i-1]
            if pos[counter] < self.minpos[i-1]:
                pos[counter] = self.minpos[i-1]

            # Allocate goal position value into byte array
            param_goal = [DXL_LOBYTE(DXL_LOWORD(vel[counter])), DXL_HIBYTE(DXL_LOWORD(vel[counter])), DXL_LOBYTE(DXL_HIWORD(vel[counter])), DXL_HIBYTE(DXL_HIWORD(vel[counter])),\
                            DXL_LOBYTE(DXL_LOWORD(pos[counter])), DXL_HIBYTE(DXL_LOWORD(pos[counter])), DXL_LOBYTE(DXL_HIWORD(pos[counter])), DXL_HIBYTE(DXL_HIWORD(pos[counter]))]
            # Add goal value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(i, param_goal)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % i)
                quit()
            counter = counter + 1

    def set_conf(self):
        # Send packets in groupSyncWrite and clears it afterward
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

    def get_pos(self, ids):
        #Uses syncread to get current position of all motors in the leg. It returns a vector with three elements
        # Parameter storage for present position was added in the constructor
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        present_position = np.zeros(len(ids),dtype = 'int')#[0 for i in ids]
        counter = 0

        for i in ids:
            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupSyncRead.isAvailable(i, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % i)
                #quit()
                continue

            # Get present position value
            present_position[counter] = self.groupSyncRead.getData(i, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            counter = counter + 1

        return present_position
        # Parameter storage for syncread is cleared in the destructor
    def torque_enable(self, ids):
         # handles scalar case
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

    def torque_disable(self, ids):
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

    def set_pos_lim(self, ids, minPosLim, maxPosLim):
        # Set pos limits for motor with ids
        # posLim registers are in the EEPROM, so the code first disable torque, the applies the change and finally enable the torque again
        # Note that if goal position is outside of the limits the motor will not move and you will get a dxl_error
        self.torque_disable(ids)

        counter = 0
        for i in ids:
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
        self.torque_enable(ids)

    def get_pos_lim(self, ids):
        # Get pos limits for motor with ids
        dxl_min_pos = np.zeros(len(ids),dtype = 'int')#[0 for i in ids]
        dxl_max_pos = np.zeros(len(ids),dtype = 'int')#[0 for i in ids]
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

    def get_status(self, ids):
        #returns status for all motors with relId
        #if dxl_status[i] is 32, motor i is in overload
        dxl_status = np.zeros(len(ids),dtype = 'int')#[0 for i in ids]
        counter = 0
        for i in ids:
            # get status
            dxl_status[counter], dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, i, ADDR_ERROR_STATUS)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            counter = counter + 1
        return dxl_status

    def reboot(self, ids):
        #applies the  reboot procedure, I always get an hardware error here and despite I clear the register the servo does not move afterward unless I turn power off
        counter = 0
        for i in ids:
            dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, i)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            counter = counter + 1
