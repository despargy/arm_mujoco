# Class Robot: to be used from other mujoco main functions.
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

class RobotGo2:
    def __init__(self):
        
        self.i_start_ctrl = 6
        self.i_end_ctrl = self.i_start_ctrl +  12 # 12 general actuators

        self.i_base_start_qpos = 6 # base position x,y,z,+quat=7
        self.i_base_end_qpos = self.i_base_start_qpos + 7
        
        self.i_start_qpos = 13
        self.i_end_qpos = self.i_start_qpos +  12 # 12 general actuators
        print("RobotGo2: i_end_qpos:", self.i_end_qpos)

        self.i_start_xpos = 0
        self.i_end_xpos = self.i_start_xpos + 7

        self.pc = np.zeros(3) #x,y,yaw
        self.xquat = np.zeros(4)  # [w, x, y, z]
        self.init_pc = np.zeros(3)
        self.init_xquat = np.zeros(4)  # [w, x, y, z]

        self.base_str = 'base'
        self.base_body_id_ = 9

    def euler_to_quat(self, x, y, z):
        """
        Parameters:
            x (float): Rotation angle around the x-axis in radians.
            y (float): Rotation angle around the y-axis in radians.
            z (float): Rotation angle around the z-axis in radians.
        Returns:
            numpy.ndarray: A quaternion represented as [w, x, y, z].
        """
        
        rot = Rotation.from_euler('xyz', [x, y, z], degrees=False)
        rot_quat = rot.as_quat(scalar_first=True)  # [w, x, y, z]
        
        
        return rot_quat
        
    def get_CoM_pos(self, data):

        self.pc = data.xpos[self.base_body_id_]
        self.xquat = data.xquat[self.base_body_id_]  # [w, x, y, z]

        return self.pc, self.xquat
    
    
    def set_CoM_pos(self, data, config):
        # Set the position of the robot's center of mass (CoM) via qpos.
        
        pos_x = config[0]
        pos_y = config[1]
        yaw = config[2]
        print("RobotGo2: set base pos:", pos_x, pos_y, yaw)
        
        pos_z = 0.35 #base height
        
        quat = self.euler_to_quat(0.0, 0.0, yaw)  #[w, x, y, z]
        
        # [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z]
        new_base_state = np.concatenate(([pos_x, pos_y, pos_z], quat))
        
        data.qpos[self.i_base_start_qpos:self.i_base_end_qpos] = new_base_state
        
        # print("RobotGo2: set base qpos:", data.qpos[self.i_base_start_qpos:self.i_base_end_qpos])