from abc import abstractmethod

from .interface import RobotInOut
from controller import Robot

class SimRobot(RobotInOut):
    def __init__(self,
                motor_list,
                 time_step=None):
        self.robot = Robot()

        super(SimRobot, self).__init__()


        if time_step is None:
            self.timestep = int(self.robot.getBasicTimeStep())
        else:
            self.timestep = time_step

        self.initialize_motors(motor_list)
        self.initialize_comms(sensor)

    def initialize_motors(self, motors_list):
        """
        initialize the motors 
        """
        self.motor =[]
        for name in motors_list:
            self.motor.append(self.robot.getDevice(name))

    def initialize_comms(self, sensor):
        """
        initialize the sensors  require to enable them and specify 
        the working frequency which is usually multiple of self.timestep
        """
        pass
   
    def torque_disable(self, motor_id):
        self.motor[motor_id].setTorque(0.0)      
    