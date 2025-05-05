import mujoco
import jax
import jax.numpy as jp

class ConfigGenerator:
    def __init__(self, data, model, inner_radius=1.0, outer_radius=2.5):
        self.arm_position = jp.zeros(2)  # JAX array instead of numpy
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.obstacle_z = 0.0
        self.data = data
        self.model = model

    def set_arm_position(self, arm_pos_quat):
        arm_pos_quat = jp.array(arm_pos_quat)
        self.arm_position = self.arm_position.at[0].set(arm_pos_quat[0])
        self.arm_position = self.arm_position.at[1].set(arm_pos_quat[1])

    def set_obstacle_position(self, obstacles_coords):
        fixed_quat = [1, 0, 0, 0]
        for i, (x, y, _) in enumerate(obstacles_coords):
            joint_name = f"obstacle{i+1}_free"
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos_index = self.model.jnt_qposadr[joint_id]
            coords = jp.array([x, y, self.obstacle_z])
            self.data.qpos = self.data.qpos.at[qpos_index: qpos_index + 7].set(
                jp.concatenate((coords, jp.array(fixed_quat)))
            )

    def generate_go2_config(self):
        r = jp.sqrt(jp.random.uniform(jax.random.PRNGKey(0), (), minval=self.inner_radius**2, maxval=self.outer_radius**2))
        arc_span = jp.deg2rad(90)
        theta_center = jp.pi / 2
        theta = jp.random.uniform(jax.random.PRNGKey(1), (), minval=theta_center - arc_span / 2, maxval=theta_center + arc_span / 2)
        pos_x = self.arm_position[0] + r * jp.cos(theta)
        pos_y = self.arm_position[1] + r * jp.sin(theta)
        yaw = theta + jp.pi
        return jp.array([pos_x, pos_y, yaw])

    def generate_obstacles(self):
        num_obstacles = 2
        obstacles = []
        for i in range(num_obstacles):
            
            rng, key = jax.random.split(rng)

            
            r = jp.sqrt(jp.random.uniform(key, (), minval=self.inner_radius**2, maxval=self.outer_radius**2))
            theta = jp.random.uniform(jax.random.PRNGKey(i+3), (), minval=0, maxval=jp.pi)
            pos_x = self.arm_position[0] + r * jp.cos(theta)
            pos_y = self.arm_position[1] + r * jp.sin(theta)
            obstacles.append((pos_x, pos_y, theta))
        self.set_obstacle_position(obstacles)
        return obstacles

    def generate_config(self, min_clearance=1.0):
        obstacles = self.generate_obstacles()
        while True:
            go2_config = self.generate_go2_config()
            go2_pos = go2_config[:2]
            distances = [jp.linalg.norm(go2_pos - jp.array(obs_pos)[:2]) for obs_pos in obstacles]
            if all(d > min_clearance for d in distances):
                break
        return go2_config, obstacles
