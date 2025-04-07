# Class Robot: to be used from other mujoco main functions.
import numpy as np
import mujoco

class RobotGo2:
    def __init__(self):
        
        self.i_start_ctrl = 6
        self.i_end_ctrl = self.i_start_ctrl +  12 # 12 general actuators

        self.i_base_start_qpos = 6 # base position x,y,z,+quat=7
        self.i_base_end_qpos = self.i_base_start_qpos + 7
        
        self.i_start_qpos = 13
        self.i_end_qpos = self.i_start_qpos +  12 # 12 general actuators

        self.i_start_xpos = 0
        self.i_end_xpos = self.i_start_xpos + 7

        self.pc = np.zeros(3)
        self.xquat = np.zeros(4)  # [w, x, y, z]

        self.base_str = 'base'
        self.base_body_id_ = 9

    def get_CoM_pos(self, data):

        self.pc = data.xpos[self.base_body_id_]
        self.xquat = data.xquat[self.base_body_id_]  # [w, x, y, z]

        return self.pc, self.xquat