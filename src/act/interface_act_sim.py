from abc import abstractmethod
from collections.abc import Iterable
from controller import Robot
from controller import Supervisor
from interface_act import RobotMotorControl

import numpy as np

class SimRobot(RobotMotorControl):
    def __init__(self,
                motor_list,
                time_step=None):
        #self.wbfnct = Robot()
        self.wbfnct = Supervisor()
        super(SimRobot, self).__init__()


        if time_step is None:
            self.timestep = int(self.wbfnct.getBasicTimeStep())
        else:
            self.timestep = time_step

        self.initialize_motors(motor_list)
        encoder_list = []
        for i in range(len(motor_list)):
            encoder_list.append('encoder'+str(int(i)))

        self.initialize_encoders(encoder_list)

    def initialize_motors(self, motors_list):
        """
        initialize the motors 
        """
        self.motor =[]
        for name in motors_list:
            self.motor.append(self.wbfnct.getDevice(name))
    
    def initialize_encoders(self, encoders_list):
        """
        initialize the encoders 
        """
        self.encoder =[]
        for name in encoders_list:
            self.encoder.append(self.wbfnct.getDevice(name))
        for i in range(len(encoders_list)):
            self.encoder[i].enable(self.timestep)
    
    def set_control_mode(self, motor_id, mode):
        pass

    def torque_disable(self, motor_id):
        if not isinstance(motor_id, Iterable):
            motor_id = [motor_id]
        for i in motor_id:
            self.motor[i].setTorque(0.0)      
    
    def get_info(self):
        pass
    
    def get_pos(self, motor_id):
        return self.encoder[motor_id].getValue()
    
    def get_pos_limits(self, motor_id):
        """
        pos limits are output in in radians using method:q_from_d
        """
        if not(isinstance(motor_id, Iterable)): 
            # se non è iterable lo si rende iterable
            motor_id = [motor_id] 
        q_min_pos = np.zeros(len(motor_id),dtype = 'int')#[0 for i in ids]
        q_max_pos = np.zeros(len(motor_id),dtype = 'int')#[0 for i in ids]
        for i in range(len(motor_id)):
            # get minPosLim
            q_min_pos[i] = self.motor[motor_id[i]].getMinPosition()
            # get maxPosLim
            q_max_pos[i] = self.motor[motor_id[i]].getMaxPosition()
        return q_min_pos, q_max_pos

    def set_vel(self,motor_id, vel):
        t0 = self.wbfnct.getTime()
        while self.wbfnct.getTime()-t0< 2:           
            self.motor[motor_id].setVelocity(abs(vel))
            if self.wbfnct.step(self.timestep)== -1:
                quit()        
                
    def set_pos(self, motor_id, vel, pos):
        #t0 = self.wbfnct.getTime()
        #while self.wbfnct.getTime()-t0< 2:
        self.motor[motor_id].setPosition(pos) 
        self.motor[motor_id].setVelocity(abs(vel))
        if np.abs(self.get_pos(motor_id)-pos)< 1e-2:
            print("Position reached with accuracy 1e-2 rad at time ", self.wbfnct.getTime())
            #break
        #if self.wbfnct.step(self.timestep)== -1:
            #quit()            
            
    def set_pos_limits(self):
        pass
    def torque_enable(self):
        pass
    
    def write_trail_field(self, n_pt):
        #If TRAIL exists in the world then silently remove it.
        try:
            existing_trail = self.wbfnct.getFromDef("draw_trj") #geometry of shape draw_trj
            existing_trail.remove()
        except:
            pass
        
        trail_string = "";  # Initialize a big string which will contain the TRAIL node.

        #Create the TRAIL Shape.
        trail_string += "DEF draw_trj Shape {\n"
        trail_string +="  appearance Appearance {\n"
        trail_string +="    material Material {\n"
        trail_string +="      diffuseColor 0 1 0\n"
        trail_string +="      emissiveColor 0 1 0\n"
        trail_string +="    }\n"
        trail_string +="  }\n"
        trail_string +="  geometry DEF TRJ_LINE_SET IndexedLineSet {\n"
        trail_string +="    coord Coordinate {\n"
        trail_string +="      point [\n"
        for i in range(n_pt): 
            trail_string+= "      0 0 0\n"
        trail_string += "      ]\n"
        trail_string += "    }\n"
        trail_string += "    coordIndex [\n"
        for i in range(n_pt):
            trail_string += "      0\n"
        trail_string += "    ]\n"
        trail_string += "  }\n"
        trail_string += "}\n"

        # Import TRAIL and append it as the world root nodes.
        root_children_field = self.wbfnct.getRoot().getField("children")
        root_children_field.importMFNodeFromString(-1, trail_string)
        
    def draw_trajectory(self, T):
        self.write_trail_field(len(T[0,:]))
        line_geom_node =self.wbfnct.getFromDef("TRJ_LINE_SET") #geometry of shape draw_trj
        coord_node = line_geom_node.getField("coord").getSFNode()
        point_field = coord_node.getField("point")
        pt_index_field = line_geom_node.getField("coordIndex")

        for i in range(len(T[0,:])):                
            point_field.setMFVec3f(i, list(T[:,i]/100)) #points in meters
            # Update the line set indices.
            #Link successive indices.
            pt_index_field.setMFInt32(i, i-1)
            if i == len(T[0,:])-1:
                pt_index_field.setMFInt32(i, -1)

            

        