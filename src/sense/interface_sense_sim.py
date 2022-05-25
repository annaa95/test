from abc import abstractmethod
from inspect import getfile
from math import degrees
from interface_sense import RobotSensorsDrive
from controller import Supervisor
from scipy.spatial.transform import Rotation as R
import numpy as np

class SimRobot(RobotSensorsDrive):
    def __init__(self,
                sensor_name,
                 time_step=None):
        super(SimRobot, self).__init__()
        self.wbfnct = Supervisor()

        if time_step is None:
            self.timestep = int(self.wbfnct.getBasicTimeStep())
        else:
            self.timestep = time_step

        self.initialize_comms(sensor_name)

    def initialize_comms(self, sensor):
        """
        initialize the sensors  require to enable them and specify 
        the working frequency which is usually multiple of self.timestep
        """
        if sensor == "ee_pose_sensor":
            self.sensor_device = self.wbfnct.getDevice(sensor)
            self.sensor_device.enable(5*self.timestep)
            self.sensor_device2 = self.wbfnct.getDevice(sensor+"2")
            self.sensor_device2.enable(5*self.timestep)

        else:
            self.sensor_device = self.wbfnct.getDevice(sensor)
            self.sensor_device.enable(self.timestep)

    def get_ee_pose(self, pos):
        if pos<3:
            #p = self.ee_elb.getSFNode().getField("translation").getSFVec3f()
            p = self.sensor_device.getValues()[pos]
            #self.wbfnct.step(self.timestep)
            return p
        else:
            #rotation = self.ee_elb.getSFNode().getField("rotation").getSFRotation()
            #rot = np.asarray(rotation[0:3])*rotation[3]
            r = self.sensor_device2.getRollPitchYaw()#R.from_rotvec(rot).as_euler("xyz")# roll pitch yaw in radians, relative to fixed frame
            return r[pos-3]

    def IMU_read_from_dev(self, reg_addr, length):
        pass
    
    def IMU_write_to_dev(self, reg_addr, length, data):
        pass

    def adc_convert_ampere(self, decimal_figures=2):
        pass

    def adc_convert_volts(self, decimal_figures=2):
        pass

    def adc_convert_battery(self, decimal_figures=2):
        pass

    def adc_read_channel(self, channel):
        pass

    def camera_frame_read(self):
        pass

    def camera_init(self, params):
        pass

    def get_info(self):
        pass

    def leak_read_all(self, id_leak):
        pass
    
    def leak_read_id(self, id_leak):
        pass

    def pressure_read(self):
        pass
    
    def temperature_read(self):
        pass
     
    