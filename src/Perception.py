import cv2
import mujoco
import numpy as np

WIDTH = 640
HEIGHT = 480

class Perception:
    def __init__(self):
        pass
    

    def get_rgbd(self, model: mujoco.MjModel, data: mujoco.MjData, context: mujoco.MjrContext):
        # === Allocate buffers ===
        rgb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        depth = np.zeros((HEIGHT, WIDTH), dtype=np.float32)

        # === Create and configure camera ===
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "realsense_cam")
        if cam.fixedcamid == -1:
            raise ValueError("Camera 'realsense_cam' not found in the model.")

        # === Set up scene and options ===
        scene = mujoco.MjvScene(model, maxgeom=1000)
        opt = mujoco.MjvOption()
        mujoco.mjv_defaultOption(opt)

        # === Update scene from camera view ===
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)

        # === Render the scene ===
        viewport = mujoco.MjrRect(0, 0, WIDTH, HEIGHT)
        mujoco.mjr_render(viewport, scene, context)

        # === Read rendered buffers ===
        mujoco.mjr_readPixels(rgb, depth, viewport, context)

        # === Depth normalization (convert to 8-bit grayscale) ===
        depth_norm = np.zeros_like(depth)
        cv2.normalize(depth, depth_norm, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        depth_gray = depth_norm.astype(np.uint8)

        # === RGB (convert to BGR for OpenCV) ===
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # === Optional: Rotate image if needed ===
        rgb_bgr = cv2.rotate(rgb_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_gray = cv2.rotate(depth_gray, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # === Show windows ===
        cv2.imshow("RGB Camera View", rgb_bgr)
        cv2.imshow("Depth Map (Grayscale)", depth_gray)
        cv2.waitKey(1)  # required for OpenCV window update