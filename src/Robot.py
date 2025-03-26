# Class Robot: to be used from other mujoco main functions.
import numpy as np
import mujoco

class RobotGo2:
    def __init__(self):
        
        self.i_start_ctrl = 6
        self.i_end_ctrl = self.i_start_ctrl +  12 # 12 general actuators

