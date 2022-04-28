# Author: Anna Astolfi
from abc import ABC, abstractmethod


class RobotMotorControl(ABC):
    """
    This class represents the basic template which contains the necessary
    methods to emit motor commands. 
    """

    def __init__(self):
        pass
    
    @abstractmethod
    def torque_enable(self, motor_id):
        """
        Enable the torque on a given motor
        """
        pass

    @abstractmethod
    def torque_disable(self, motor_id):
        """
        Disable the torque on a given motor 
        """
        pass       
    
    @abstractmethod
    def get_pos(self,motor_id):
        """
        Return the current position of a given motor, in radians
        """
        pass

    @abstractmethod
    def set_pos(self, motor_id, vel, pos):
        """
        command a position to the motor identified by motor_id to be reached and a velocity to reach it
        NOTA: differenza tra simualtore e motori
        """
        pass

    @abstractmethod
    def set_pos_limits(self, motor_id):
        """
        """
        pass

    @abstractmethod
    def get_pos_limits(self, motor_id):
        """
        This function returns the motor limits of a given motor identified by motor_id. 
        If motor_id is a integer, the function convert it into a list first then outputs the min and max pos in radians.
        If motor_id ia an iterable object, the output are two arrays of the same size as motor_id, 
            one for the minpos, the other for the maxpos, in radians
        """
        pass




    @abstractmethod
    def get_info(self):
        """
         Diagnostic information mostly useful for debugging.
        """
        pass
