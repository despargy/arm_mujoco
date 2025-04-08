import numpy as np


class ConfigGenerator:
    def __init__(self, inner_radius=1.0, outer_radius=3.0):
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
        """
        Generate a random robot configuration with the go2 spawned in an annulus.
        
        The annulus is defined by the inner_radius and outer_radius, centered at the arm's position.
        
        Parameters:
            arm_pos_quat (iterable): The arm's position and quaternion (only x,y are used).
            
        Returns:
            dict: A configuration dictionary that includes the robot's x, y, and yaw.
                  Example: {"robot": {"x": 2.5, "y": 1.3, "yaw": -1.57}}
        """
        arm_pos_quat = np.array(arm_pos_quat)
        self.arm_position[0] = arm_pos_quat[0]
        self.arm_position[1] = arm_pos_quat[1]

        # Sample a random radius uniformly in area between the inner and outer radius.
        r = np.sqrt(np.random.uniform(self.inner_radius**2, self.outer_radius**2))
        theta = np.random.uniform(0, 2 * np.pi)

        pos_x = self.arm_position[0] + r * np.cos(theta)
        pos_y = self.arm_position[1] + r * np.sin(theta)
        yaw = np.random.uniform(-np.pi, np.pi)

        # Create and return the configuration dictionary.
        return {"robot": {"x": pos_x, "y": pos_y, "yaw": yaw}}

