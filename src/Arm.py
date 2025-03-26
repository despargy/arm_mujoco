# Class Arm: to be used from other mujoco main functions.
import numpy as np
import mujoco

class Arm:
    def __init__(self, model, data):

        # Init variables
        self.q_desired = np.array([-0.75, -1.57, 1.57, -0.37, -2.45, -2.45])
        self.t_init = 3.0
        self.kp = 15
        self.freq = 2.0
        self.Ax, self.Ay, self.Az = 0.01, 0.01, 0.05
        self.dt = model.opt.timestep
        # Position variables
        self.p0 = np.zeros(3)
        self.q_out = np.zeros(6)
        self.dq_out = np.zeros(6)

        self.p_c = np.zeros(3)
        self.dp_c = np.zeros(3)
        self.p_d = np.zeros(3)
        self.dp_d = np.zeros(3)

        self.ep = np.zeros(3)

        # End-effector body id
        self.ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
        if self.ee_body_id == -1:
            raise  ValueError("Invalid end-effector body ID (ee_body_id).")
        
        self.J = None
        self.t = 0.0
        # Define indexing for ctrl
        self.i_start_ctrl = 0
        self.i_end_ctrl = self.i_start_ctrl + 6
        # Define indexing for qpos
        self.i_start_qpos = 0
        self.i_end_qpos = self.i_start_qpos + 6


    def control_Cb(self, model, data):
        self.t = data.time - self.t_init

        if data.time < self.t_init:
            data.ctrl[self.i_start_ctrl:self.i_end_ctrl] = self.q_desired
            self.q_out[:] = data.qpos[self.i_start_qpos:self.i_end_qpos]
            self.p0[:] = data.xpos[self.ee_body_id]
        else:
            jacp = np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jacp, None, self.ee_body_id)
            Jall = jacp
            self.J = Jall[:3, :6]  # Only the first 3 rows (linear velocity)
            # Pseudo-inverse
            U, S, Vt = np.linalg.svd(self.J, full_matrices=False)
            S_inv = np.array([1/s if s > 1e-2 else 0.0 for s in S])
            J_pinv = Vt.T @ np.diag(S_inv) @ U.T

            # Actual position and velocity
            self.p_c = data.xpos[self.ee_body_id]
            self.dp_c = data.cvel[self.ee_body_id][:3]  # local linear vel

            # Desired trajectory
            self.p_d = self.p0 + np.array([
                self.Ax * np.sin(self.freq * self.t),
                self.Ay * np.sin(self.freq * self.t),
                self.Az * np.sin(self.freq * self.t)
            ])
            self.dp_d = np.array([
                self.Ax * self.freq * np.cos(self.freq * self.t),
                self.Ay * self.freq * np.cos(self.freq * self.t),
                self.Az * self.freq * np.cos(self.freq * self.t)
            ])

            # Error and control
            self.ep = self.p_c - self.p_d
            self.dq_out = J_pinv @ (-self.kp * self.ep)
            self.q_out += self.dq_out * self.dt

            # Send general actuator commands - Listen as joint position...
            data.ctrl[self.i_start_ctrl:self.i_end_ctrl] = self.q_out
