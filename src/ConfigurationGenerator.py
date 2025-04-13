import numpy as np


class ConfigGenerator:
    def __init__(self, inner_radius=1.0, outer_radius=2.5):
        """
        Initialize the configuration generator for the robot.

        Parameters:
            inner_radius (float): Minimum distance from the arm (restricted zone).
            outer_radius (float): Maximum distance from the arm for spawning.
        """
        self.arm_position = np.arange(2)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius


    def set_arm_position(self, arm_pos_quat):
        
        arm_pos_quat = np.array(arm_pos_quat)
        
        self.arm_position[0] = arm_pos_quat[0]
        self.arm_position[1] = arm_pos_quat[1]
        
        
    def generate_go2_config(self):
        r = np.sqrt(np.random.uniform(self.inner_radius**2, self.outer_radius**2))

        arc_span = np.deg2rad(90)        
        theta_center = np.pi / 2          # facing "forward" (in +y)
        theta = np.random.uniform(theta_center - arc_span / 2, theta_center + arc_span / 2)

        pos_x = self.arm_position[0] + r * np.cos(theta)
        pos_y = self.arm_position[1] + r * np.sin(theta)

        yaw = theta + np.pi               # face toward the arm
        
        go2_config = np.array([pos_x, pos_y, yaw])
        
        return go2_config
    
    def generate_obstacles(self):
        """
        Generate obstacle configurations around the arm.
        Returns:
            List of obstacle configurations.
        """
        num_obstacles = np.random.randint(1, 3)  # Random number of obstacles between 1 and 3
        obstacles = []

        for _ in range(num_obstacles):
            r = np.sqrt(np.random.uniform(self.inner_radius**2, self.outer_radius**2))
            theta = np.random.uniform(0, 2 * np.pi)

            pos_x = self.arm_position[0] + r * np.cos(theta)
            pos_y = self.arm_position[1] + r * np.sin(theta)

            obstacles.append((pos_x, pos_y))

        return obstacles
    
    
    def generate_config(self):
        obstacles = self.generate_obstacles()
        
        min_clearance = 0.3 #minimum distance to obstacle

        # find a valid go2_config(it has to be greater than 0.3 from all obstacles)
        while True:
            
            # generate go2 position
            go2_config = self.generate_go2_config()
            
            go2_pos = go2_config[:2]  # (x, y)

            # compute all distances from go2 to all obstacles
            distances = [np.linalg.norm(go2_pos - np.array(obs_pos)) for obs_pos in obstacles]
            
            if all(d > min_clearance for d in distances):
                break  # Valid placement found

        return go2_config, obstacles
