from abc import ABC, abstractmethod


class RobotPerceive(ABC):
    """
    This class represents the basic template which contains the necessary
    methods to acquire sensors data. 
    """

    def __init__(self):
        pass
    
    @abstractmethod
    def adc_read_channel(self, channel):
        """
        Function to read SPI data from MCP3008 chip
        Channel must be an integer 0-7        
        """
        pass

    @abstractmethod
    def adc_convert_volts(self, decimal_figures = 2):
        """
        Function to convert data to voltage level,
        rounded to specified number of decimal places.        
        """
        pass       
    
    @abstractmethod
    def adc_convert_ampere(self,decimal_figures = 2):
        """
        Function to convert data to current level,
        rounded to specified number of decimal places.        
        """
        pass

    @abstractmethod
    def adc_convert_battery(self, decimal_figures =2):
        """
        Function to convert data to battery charge estimation
        the output is the percentage of charge
        """
        pass

    @abstractmethod
    def IMU_write_to_dev(ser, reg_addr, length, data):
        """
        Write data to IMU
        """
        pass

    @abstractmethod 
    def IMU_read_from_dev(ser, reg_addr, length):
        """
        Read data from IMU        
        """
        pass
    
    @abstractmethod
    def leak_read_id(self, id_leak):
        """
        Read a the status of leakage sensors specified by id_leak
        """
        pass

    @abstractmethod
    def leak_read_all(self, id_leak):
        """
        Returns whether there is a leakage in the system, regardless the position
        """
        pass

    @abstractmethod
    def pressure_read(self):
        """
        Returns the environment pressure 
        """
        pass

    @abstractmethod
    def temperature_read(self):
        """
        Returns the environment temperature 
        """
        pass

    @abstractmethod
    def camera_init(self, params):
        """
        pass the parameters for camera initialization (resolution, size ...) 
        """
        pass
    @abstractmethod
    
    def camera_frame_read(self):
        """
        Returns the image from the camera 
        """
        pass
    @abstractmethod
    def get_info(self):
        """
         Diagnostic information mostly useful for debugging.
        """
        pass
