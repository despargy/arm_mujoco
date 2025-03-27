import cv2
import mujoco
import numpy as np

class Perception:
    def __init__(self, height=480, width=640):
        self.width = width
        self.height = height

    def get_rgbd(self, model: mujoco.MjModel, data: mujoco.MjData, context: mujoco.MjrContext):
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.zeros((self.height, self.width), dtype=np.float32)

        # Configure camera (robot-mounted)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "realsense_cam")
        if cam.fixedcamid == -1:
            raise ValueError("Camera 'realsense_cam' not found in the model.")

        # Local scene + options
        scene = mujoco.MjvScene(model, maxgeom=1000)
        opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(opt)

        # Update scene for the robot camera
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)

        # Switch to offscreen buffer
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)

        # Render into the offscreen buffer
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        mujoco.mjr_render(viewport, scene, context)

        # Read out the rendered RGB and depth data
        mujoco.mjr_readPixels(rgb, depth, viewport, context)

        # Switch back to the visible/window buffer
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, context)

        # Process depth for display
        depth_norm = np.zeros_like(depth)
        cv2.normalize(depth, depth_norm, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        depth_gray = depth_norm.astype(np.uint8)

        # Convert RGB to BGR for OpenCV
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Optional rotation
        rgb_bgr = cv2.rotate(rgb_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_gray = cv2.rotate(depth_gray, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Display in separate OpenCV windows
        cv2.imshow("RGB Camera View", rgb_bgr)
        cv2.imshow("Depth Map (Grayscale)", depth_gray)
        cv2.waitKey(1)