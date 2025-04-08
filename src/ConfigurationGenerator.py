import numpy as np


class ConfigGenerator:
    def __init__(self, inner_radius=0.5, outer_radius=3.0):
        """
        Initialize the configuration generator for the robot.

        Parameters:
            inner_radius (float): Minimum distance from the arm (restricted zone).
            outer_radius (float): Maximum distance from the arm for spawning.
        """
        self.arm_position = [0.0, 0.0]
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius

    def generate_config(self, arm_pos_quat):
       
        arm_pos_quat = np.array(arm_pos_quat)
        self.arm_position[0] = arm_pos_quat[0]
        self.arm_position[1] = arm_pos_quat[1]

        # Sample a random radius uniformly in area between the inner and outer radius.
        r = np.sqrt(np.random.uniform(self.inner_radius**2, self.outer_radius**2))
        theta = np.random.uniform(0, 2 * np.pi)
        print("r",r)

        pos_x = self.arm_position[0] + r * np.cos(theta)
        pos_y = self.arm_position[1] + r * np.sin(theta)
        yaw = np.random.uniform(-np.pi, np.pi)

        # Create and return the configuration dictionary.
        return {"robot": {"x": pos_x, "y": pos_y, "yaw": yaw}}

