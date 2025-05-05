import numpy as np
import mujoco
# from utilities import euler_to_quat

class ConfigGenerator:
    def __init__(self,data,model, inner_radius=1.0, outer_radius=2.5):
        """
        Initialize the configuration generator for the robot.

        Parameters:
            inner_radius (float): Minimum distance from the arm (restricted zone).
            outer_radius (float): Maximum distance from the arm for spawning.
        """
        self.arm_position = np.arange(2)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.obstacle_z = 0.0
        self.data= data
        self.model = model


    def set_arm_position(self, arm_pos_quat):
        
        arm_pos_quat = np.array(arm_pos_quat)
        
        self.arm_position[0] = arm_pos_quat[0]
        self.arm_position[1] = arm_pos_quat[1]
        
    def set_obstacle_position(self, obstacles_coords):
        
        fixed_quat = [1, 0, 0, 0]


        for i, (x, y, _) in enumerate(obstacles_coords):
            
            joint_name = f"obstacle{i+1}_free"
            
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos_index = self.model.jnt_qposadr[joint_id]

            coords = np.array([x, y, self.obstacle_z])
            self.data.qpos[qpos_index: qpos_index + 7] = np.concatenate((coords, fixed_quat))

            
    def generate_go2_config(self):
        r = np.sqrt(np.random.uniform(self.inner_radius**2, self.outer_radius**2))

        arc_span = np.deg2rad(90)        
        theta_center = np.pi / 2   
    # facing "forward" (in +y)
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
        # num_obstacles = np.random.randint(1, 3)  # Random number of obstacles between 1 and 3
        num_obstacles = 2
        obstacles = []
        
        
        for _ in range(num_obstacles):
            
            r = np.sqrt(np.random.uniform(self.inner_radius**2, self.outer_radius**2))
            theta = np.random.uniform(0, np.pi)

            pos_x = self.arm_position[0] + r * np.cos(theta)
            pos_y = self.arm_position[1] + r * np.sin(theta)

            obstacles.append((pos_x, pos_y,theta))
            
        self.set_obstacle_position(obstacles)

        return obstacles
    
    
    def generate_config(self, min_clearance = 1.0):
        obstacles = self.generate_obstacles()
        
        # find a valid go2_config(it has to be greater than 0.3 from all obstacles)
        while True:
            
            # generate go2 position
            go2_config = self.generate_go2_config()
            
            go2_pos = go2_config[:2]  # (x, y)

            # compute all distances from go2 to all obstacles
            distances = [np.linalg.norm(go2_pos - np.array(obs_pos)[:2]) for obs_pos in obstacles]
            
            if all(d > min_clearance for d in distances):
                break  # Valid placement found

        return go2_config, obstacles
