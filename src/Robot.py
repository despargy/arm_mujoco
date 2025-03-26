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


