# Author: Anna Astolfi
from abc import ABC, abstractmethod
import numpy as np

class RobotAbc():
        """
        This class represents the basic template which contains the necessary
        methods for the robot to elaborate information to be sent to an output device.
        These include single-leg kinematics (fk, ik), multi-leg kinematics (stewart), 
        computation of workspace and admissibility tests
        """
        def __init__(self, 
                    leg_num = 6, 
                    leg_joints = 3, 
                    ): 
                #Structural characteristics of the robot.
                self.leg_num = leg_num			        #Number of legs
                self.leg_joints = leg_joints	        #Number of joints in a leg
                self.joint_num = leg_num*leg_joints     #Number of joints in the robot
                
                #Lengths of leg segments in cm
                
                #Dimensions of body in cm
                
                #[min,max] range of joints positioning in rad.

                # Modern Robotics approach requirements: 
                
                #   Mhome configurations (position of end effector when all joint are at 0 position)
                #       each row is a leg
                #       RESHAPE TO USE M_n:  row_n = [n0, n1,...n15] -> M_n = np.transpose(np.reshape(row_n, [4,4])) 
                self.home_matrices = np.zeros([self.leg_num,16]) 
                #   Sn = (ωn, vn) is the screw axis of joint n as expressed in the fixed base frame 6x1
                self.screw_axis = np.zeros([6,self.joint_num])              

                #Create point cloud of leg workspace for admissibility test.
                nsample = 11
                self.compute_workspace(nsample)

                # Goto pose just after creation
        
        @abstractmethod                
        def __del__(self):
                pass

        def compute_workspace(self, nsamples= 11):
                self.cloud = np.zeros([nsamples**3,3])

                c = 0
                q1 = np.linspace(self.q_min[0], self.q_max[0], nsamples)
                q2 = np.linspace(self.q_min[1], self.q_max[1], nsamples)
                q3 = np.linspace(self.q_min[2], self.q_max[2], nsamples)
                for i in range(nsamples):
                        for j in range(nsamples):
                                for k in range(nsamples):
                                        self.cloud[c,:] = self.kine([q1[i], q2[j], q3[k]],1) #patch rapida. Checko le gambe 3,4,5 sul workspace delle 1,2,6
                                        c = c+1
                

        @abstractmethod                        
        def fwd_kine(self, q):
                """
                This function implements the single leg direct kinematics as Product of Exponentials-PoE
                Take as inputs the joint angles and outputs the End Effector cartesian position in body Reference
                Frame.
                """
                pass
        
        @abstractmethod
        def inv_kine(self, p):
                """
                This function implements the single leg inverse kinematics as Logarithm 
                Take as inputs the  End Effector cartesian position in body Reference Frame and outputs joint angles
                """
                pass
        
        @abstractmethod
        def admissible(self, T):
                """
                Test if points in `T` are in `hull`

                `T` should be a `NxK` coordinates of `N` points in `K` dimensions
                `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the
                coordinates of `M` points in `K`dimensions for which Delaunay triangulation
                will be computed
                """
                pass
        
        @abstractmethod
        def cartesian2joint_traj(self, T, dT):
                """
                Traduzione di una traiettoria T = [x,y,z](t) Nx3 (with N time frames, and 3 coordinate end effectors)
                with dT = time to complete the trjectory in seconds
                in angoli di giunto Q[rad], velocità Qdot [rad/s] e check ammissibilità
                """
                pass
                
        def get_leg_slice(self, data, leg_id):
                #Data: array of size leg_joints*joint_num
                #leg_id: id of a leg
                #return: a slice of data corresponding to the joints in leg with id leg_id
                #leg_id=1--->[data[1], data[2], data[3]]
                return data[self.leg_joints*(leg_id-1):self.leg_joints*(leg_id-1)+self.leg_joints]

        def goto_pose(self, vel, pose):
                for i in range(1,self.leg_num+1):
                        joint_ids = self.get_leg_slice(self.motor_ids,i)
                        joint_vel = self.profvel_from_qdot(self.get_leg_slice(static_poses_vel[vel],i))
                        joint_pos = self.d_from_q(joint_ids,self.get_leg_slice(static_poses_pos[pose],i))
                        
                        self.Q_cur[3*(i-1):3*(i-1)+3] = np.radians(self.get_leg_slice(static_poses_pos[pose],i))
                        self.robot_angles_and_status.publish(list(self.Q_cur),list(self.joints_status))

                        if self.phantom == 0:
                                self.motor_lock.acquire()
                                self.motorHandler.allocate_conf(joint_ids, joint_vel, joint_pos)
                                self.motorHandler.set_conf()
                                self.motor_lock.release()
                        
        def change_configuration(self, nq):
            nq = nq*np.pi/180
            oq = self.get_pose(np.arange(0,18))
            gait_t = 2 #[s]
            #assumption: configuration with neutral asset <--> all legs have the same height when they touch the ground
            new_leg_heights = np.zeros(6)
            old_leg_heights = np.zeros(6)
            for i in np.arange(0,6):
                np_i = self.kine(nq[3*i:3*i+3],i) #possibile fonte di errore
                op_i = self.kine(oq[3*i:3*i+3],i)
                new_leg_heights[i] = np_i[2]
                old_leg_heights[i] = op_i[2]
            nh = min(new_leg_heights)
            oh = min(old_leg_heights)
            if np.allclose(nq, oq):
                print("same pose as before")
                return True
            elif np.abs(nh-oh)<0.1:
                n1 = 0
                n2 = 15
            else:
                n1 = 30
                n2 = 15
            n = n1 + 2*n2
            t = np.linspace(0,1,n2)
            delta_t = gait_t/n
            Q = np.zeros([18,n])
            Qdot = np.zeros([18,n])
            admiss = np.ones(6, dtype=bool)
            
            for i in np.arange(0,6):
                nq_i = nq[3*i:3*i+3] #new configuration
                oq_i = oq[3*i:3*i+3] #old configuration
                np_i = self.kine(nq_i,i) #new foot tip position
                op_i = self.kine(oq_i,i) #old foot tip position
                #FIRST PHASE--> All legs go to new height with linear trajectory
                xi_1 = np.ones(n1)*op_i[0]
                yi_1 = np.ones(n1)*op_i[1]
                zi_1 = np.linspace(op_i[2], np_i[2], n1)
                #SECOND PHASE-->Tripod1 goes to new feet position with circumference arcs, Tripod2 stays
                if i==0 or i==2 or i==4:
                    xi_2 = np.linspace(op_i[0], np_i[0], n2)
                    yi_2 = np.linspace(op_i[1], np_i[1], n2)
                    zi_2 = np_i[2] + 5*np.sin(np.pi*t)
                else:
                    xi_2 = np.ones(n2)*op_i[0]
                    yi_2 = np.ones(n2)*op_i[1]
                    zi_2 = np.ones(n2)*np_i[2]
                #THIRD PHASE --> Tripod1 stays, Tripod2 goes to new feet position with circumference arcs
                if i==0 or i==2 or i==4:
                    xi_3 = np.ones(n2)*np_i[0]
                    yi_3 = np.ones(n2)*np_i[1]
                    zi_3 = np.ones(n2)*np_i[2]
                else:
                    xi_3 = np.linspace(op_i[0], np_i[0], n2)
                    yi_3 = np.linspace(op_i[1], np_i[1], n2)
                    zi_3 = np_i[2] + 5*np.sin(np.pi*t)
                # merge phases
                x_i = np.hstack((xi_1,xi_2,xi_3))
                y_i = np.hstack((yi_1,yi_2,yi_3))
                z_i = np.hstack((zi_1,zi_2,zi_3))
                T_i = np.vstack((x_i,y_i,z_i))
                # check admissibility and compute inverse kinematics and joint velocities
                admiss[i] = self.admissible(np.transpose(T_i))
                if admiss[i]:
                    for j in range(0,n):
                        Q[3*i:3*i+3,j] = self.leg_inv_kine(T_i[:,j], i) # leg_id=1...se metto i non funziona. C'è qualche incongruenza di segno sulle cinematiche
                    Qdot[3*i:3*i+3,0] = Q[3*i:3*i+3,n-1]-Q[3*i:3*i+3,0]
                    Qdot[3*i:3*i+3,1:n] = np.abs(np.diff(Q[3*i:3*i+3,0:n])/delta_t)
                else:
                    print('non admissible trj for leg_id: '+ str(i))
            # execute trajectory or terminate if trajectory is not admissible
            if all(admiss):
                for k in range(0, n):
                    self.move_all_legs(Qdot[:,k]*180/np.pi, Q[:,k]*180/np.pi)
                    time.sleep(delta_t)
                print("successfully changed pose")
                return True
            else:
                print("non admissible trajectories in transition")
                return False

        def move_all_legs(self, vel, pos):
                #Marce's version (Sept 2019)
                joint_vel = self.profvel_from_qdot(vel)
                joint_pos = self.d_from_q(self.motor_ids[0:18],pos)
                for c in range(0,6):

                        self.Q_cur[3*c:3*c+3] = np.radians(pos[c*3:c*3+3:1])
                        self.joints_status = np.ones(18, dtype=int)
                        self.robot_angles_and_status.publish(list(self.Q_cur),list(self.joints_status))
                        
                        if self.phantom == 0: 
                                self.motor_lock.acquire()
                                self.motorHandler.allocate_conf(self.motor_ids[c*3:c*3+3:1], joint_vel[c*3:c*3+3:1], joint_pos[c*3:c*3+3:1])
                                self.motorHandler.set_conf()
                                self.motor_lock.release()
                                        
        def push(self, leg_ids, joint_vel, q_femur, q_tibia, underactuated):
                #q_femur, q_tibia [deg] - final position, array of length = length(leg_ids)
                #joint_vel [digit] - angular velocity of joints, array of length = length(leg_ids)
                #underactuated [binary] - if 1 femur off
                coxa_joint_ids = leg_ids*3-2
                femur_joint_ids = leg_ids*3-1
                tibia_joint_ids = leg_ids*3
                print(self.phantom)
                for i in np.arange(0,len(leg_ids)): 
                        if self.phantom == 0: # sending motor commands to robot
                                self.motor_lock.acquire()
                                if underactuated:
                                        #print('switching off motors', femur_joint_ids[i])
                                        self.torque_disable(femur_joint_ids[i]) #the disable function manages the phantom case automatically
                                self.motorHandler.allocate_conf(femur_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(femur_joint_ids[i], q_femur[leg_ids[i]-1]))
                                self.motorHandler.allocate_conf(tibia_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(tibia_joint_ids[i], q_tibia[leg_ids[i]-1]))
                                self.motorHandler.set_conf()
                                self.motor_lock.release()

                        # PHANTOM : set active joints status to 1
                        self.joints_status[coxa_joint_ids[i]-1]=1
                        self.joints_status[tibia_joint_ids[i]-1]=1
                        # PHANTOM : set motor angles 
                        self.Q_cur[femur_joint_ids[i]-1] = np.radians(q_femur[leg_ids[i]-1]) #ignored if status ==0
                        self.Q_cur[tibia_joint_ids[i]-1] = np.radians(q_tibia[leg_ids[i]-1])
                        self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                      

        def pull(self, leg_ids, joint_vel, q_coxa, q_femur, q_tibia, underactuated):
                #q_femur, q_tibia, q_coxa [deg] - final position, array of length = length(leg_ids)
                #joint_vel [digit] - angular velocity of joints, array of length = length(leg_ids)
                # during pull there is the previously active tripod becoming idle and the other one preparing for landing
                coxa_joint_ids = leg_ids*3-2
                femur_joint_ids = leg_ids*3-1
                tibia_joint_ids = leg_ids*3

                for i in np.arange(0,len(leg_ids)): #: np.array([3,6])-1 per prove banco
                        if underactuated: # se fully actuted non succede nulla perchè è già tutto acceso
                                self.torque_enable(femur_joint_ids[i])
                                #print('switching on motors', femur_joint_ids[i])
                        if self.phantom == 0:
                                self.motor_lock.acquire()
                                self.motorHandler.allocate_conf(femur_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(femur_joint_ids[i], q_femur[leg_ids[i]-1]))
                                self.motorHandler.allocate_conf(tibia_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(tibia_joint_ids[i], q_tibia[leg_ids[i]-1]))
                                self.motorHandler.allocate_conf(coxa_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(coxa_joint_ids[i], q_coxa[leg_ids[i]-1]))
                                self.motorHandler.set_conf()
                                self.motor_lock.release()
                        # PHANTOM: set motor angles (DONE ANYWAY)
                        self.Q_cur[femur_joint_ids[i]-1] = np.radians(q_femur[leg_ids[i]-1])
                        self.Q_cur[tibia_joint_ids[i]-1] = np.radians(q_tibia[leg_ids[i]-1])
                        self.Q_cur[coxa_joint_ids[i]-1] = np.radians(q_coxa[leg_ids[i]-1])
                        self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                        
        def set_coxa(self, leg_ids, joint_vel, joint_pos):
                #..._joint_vel [digit]
                #..._joint_pos [deg]
                #SET COXA ACTION: set coxa joint at joint_pos position with joint_vel velocity
                joint_ids = leg_ids*3-2
                self.Q_cur[joint_ids-1] = np.radians(joint_pos)
                self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                if self.phantom == 0:
                        self.motor_lock.acquire()
                        self.motorHandler.allocate_conf(joint_ids, joint_vel, self.d_from_q(joint_ids, joint_pos))
                        #set positions previously allocated
                        self.motorHandler.set_conf()
                        self.motor_lock.release()

        def ik_stewart(self, displacement, orientation, F_b):
                rotation = eul.euler2mat(orientation[0],orientation[1],orientation[2],axes='szyx')
                #Anchor points of legs in the base frame
                A_b = np.tile(displacement,self.leg_num) + np.reshape(rotation.dot(self.leg_frame_b.T).T,18) 
                #Feet tip position in leg frames
                F_l = rotation.T.dot(np.reshape(F_b - A_b,(6,3)).T) 			
                #Mirror left side legs.
                F_l[0,self.leg_num//2:self.leg_num + 1] = -F_l[0,self.leg_num//2:self.leg_num + 1]
                F_l[1,self.leg_num//2:self.leg_num + 1] = -F_l[1,self.leg_num//2:self.leg_num + 1]
                return F_l.T

        def manipulate(self, vel, pos):
                self.motor_lock.acquire()
                self.motorHandler.allocate_conf([19], vel, pos)
                #set positions previously allocated
                self.motorHandler.set_conf()
                self.motor_lock.release()
