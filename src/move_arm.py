import mujoco
import glfw
import numpy as np
import cv2
import csv
from Arm import Arm
from Robot import RobotGo2
from Perception import Perception

# ========== Paths ==========
XML_PATH = "../xml/scene.xml"
DATA_LOG = "data.csv"

# ========== Load Model ==========
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# ========== Components ==========
arm = Arm(model=model, data=data)
robot_go2 = RobotGo2()
perception = Perception()

# ========== Logging ==========
csv_file = open(DATA_LOG, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["time", "ep_x", "ep_y", "ep_z", "p_cx", "p_cy", "p_cz", "p_dx", "p_dy", "p_dz"])

# ========== GLFW Setup ==========
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(1244, 700, "MuJoCo Arm Control", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# ========== Visualization Setup ==========
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scene = mujoco.MjvScene(model, maxgeom=2000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
perception_context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)

mujoco.mjv_defaultCamera(cam)
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
mujoco.mjv_defaultOption(opt)

cam.azimuth, cam.elevation, cam.distance = -90.48, -40.10, 3.21
cam.lookat[:] = [-0.047404, -0.001591, 0.330533]

# ========== Mouse Handling ==========
mouse = {"left": False, "middle": False, "right": False, "last_x": 0.0, "last_y": 0.0}

def mouse_button_callback(window, button, action, mods):
    mouse["left"] = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    mouse["middle"] = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
    mouse["right"] = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
    mouse["mods"] = mods
    mouse["last_x"], mouse["last_y"] = glfw.get_cursor_pos(window)

def cursor_pos_callback(window, xpos, ypos):
    if not (mouse["left"] or mouse["middle"] or mouse["right"]):
        return
    dx = xpos - mouse["last_x"]
    dy = ypos - mouse["last_y"]
    mouse["last_x"], mouse["last_y"] = xpos, ypos
    width, height = glfw.get_window_size(window)
    dx /= height
    dy /= height
    if mouse["right"]:
        action = mujoco.mjtMouse.mjMOUSE_MOVE_H if (mouse["mods"] & glfw.MOD_SHIFT) else mujoco.mjtMouse.mjMOUSE_MOVE_V
    elif mouse["left"]:
        action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if (mouse["mods"] & glfw.MOD_SHIFT) else mujoco.mjtMouse.mjMOUSE_ROTATE_V
    elif mouse["middle"]:
        action = mujoco.mjtMouse.mjMOUSE_ZOOM
    else:
        return
    mujoco.mjv_moveCamera(model, action, dx, dy, scene, cam)

def scroll_callback(window, xoffset, yoffset):
    mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, scene, cam)

# Register callbacks
glfw.set_mouse_button_callback(window, mouse_button_callback)
glfw.set_cursor_pos_callback(window, cursor_pos_callback)
glfw.set_scroll_callback(window, scroll_callback)

# ========== Control Logic ==========
def my_controller(model, data):
    #Cb for periodic motion
    arm.control_Cb(model=model, data=data)

    #Print joint mapping
    # for i in range(model.njnt):
    #     joint_name = model.joint(i).name
    #     dof_start = model.jnt_dofadr[i]  # Index where this joint's velocity starts in qvel
    #     dof_count = 1 if model.jnt_type[i] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE) else 6
    #     print(f"Joint {joint_name}: qvel[{dof_start}:{dof_start + dof_count}]")

    data.ctrl[robot_go2.i_start_ctrl:robot_go2.i_end_ctrl] = model.keyframe("home").ctrl[robot_go2.i_start_ctrl:robot_go2.i_end_ctrl]
    csv_writer.writerow([
        data.time,
        arm.ep[0], arm.ep[1], arm.ep[2],
        arm.p_c[0], arm.p_c[1], arm.p_c[2],
        arm.p_d[0], arm.p_d[1], arm.p_d[2]
    ])

# ========== Main Loop ==========
while not glfw.window_should_close(window):
    my_controller(model, data)

    sim_start = data.time
    while data.time - sim_start < 1.0 / 60.0:
        mujoco.mj_step(model, data)

    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    width, height = glfw.get_framebuffer_size(window)
    viewport = mujoco.MjrRect(0, 0, width, height)
    mujoco.mjr_render(viewport, scene, context)

    glfw.make_context_current(window)
    perception.get_rgbd(model, data, perception_context)
    # glfw.make_context_current(window) #maybe we can comment ths line

    glfw.swap_buffers(window)
    glfw.poll_events()

# ========== Cleanup ==========
csv_file.close()
glfw.terminate()
