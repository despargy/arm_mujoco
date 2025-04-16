import cv2
import mujoco
import numpy as np
# from mujoco_playground._src.dynamic_events.arm_mujoco.src.Tag import Tag
from Tag import Tag
from dt_apriltags import Detector
from shapely.geometry import Polygon
from math import atan2
from collections import deque
from scipy.stats import skew

def order_clockwise(points):
    # Step 1: Calculate the centroid (mean of the points)
    centroid = np.mean(points, axis=0)
    
    # Step 2: Calculate the polar angle of each point relative to the centroid
    def polar_angle(point):
        return atan2(point[1] - centroid[1], point[0] - centroid[0])
    
    # Step 3: Sort points based on the angle in counter-clockwise direction
    sorted_points = sorted(points, key=polar_angle)
    
    # Step 4: Reverse the sorted points to get the clockwise order
    sorted_points.reverse()

    return sorted_points

class Perception:
    def __init__(self, height=480, width=640):
        self.width = width
        self.height = height
        self.perception_context = None
        # Optical Flow Params
        self.params_shitomasi = dict(maxCorners=10, qualityLevel=0.25, minDistance=7)
        self.params_lucas_kanade = dict(
            winSize=(5, 5),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        self.colours = np.random.randint(0, 255, (100, 3))
        self.initialized = False
        self.old_gray = None
        self.edges = None
        self.canvas = None

        # Tags releated
        self.at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        self.AllTagsDict = {}
        self.NO_TAGS = True
        self.n_tags = 0
        
        self.area_btw_tags = 0.0
        self.coords = []
        self.depth = None

    def get_rgbd(self, model: mujoco.MjModel, data: mujoco.MjData, context: mujoco.MjrContext):
        
        """Simple camera view"""
        
        
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
        
        # Convert normalized depth to grayscale
        depth_gray = depth_norm.astype(np.uint8) 

        # Convert RGB to BGR for OpenCV
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Optional rotation
        # rgb_bgr = cv2.rotate(rgb_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # depth_gray = cv2.rotate(depth_gray, cv2.ROTATE_90_COUNTERCLOCKWISE)

        rgb_bgr1 = cv2.rotate(rgb_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE) 
        self.rgb = cv2.flip(rgb_bgr1, 1)

        depth_gray1 = cv2.rotate(depth_gray, cv2.ROTATE_90_COUNTERCLOCKWISE) 
        self.depth = cv2.flip(depth_gray1, 1)

        # Display in separate OpenCV windows
        # cv2.imshow("RGB Camera View", self.rgb)
        # cv2.imshow("Depth Map (Grayscale)", self.depth)
        # cv2.waitKey(1)
        
    def _render_camera_view(self, model: mujoco.MjModel, data: mujoco.MjData, context: mujoco.MjrContext):
        
        ### init rgb and depth arrays
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

        # Update scene and render to offscreen
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        mujoco.mjr_render(viewport, scene, context)
        mujoco.mjr_readPixels(rgb, depth, viewport, context)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, context)
        
        # Process depth for display
        depth_norm = np.zeros_like(depth)
        cv2.normalize(depth, depth_norm, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        depth_gray = depth_norm.astype(np.uint8)
        
        return rgb, depth_gray
    
    def _define_AOI(self, height, width, cx, cy, w, h):
        """
        Defines an Area of Interest (AOI) mask and a rectangle overlay on a blank image.

        Parameters:
            height (int): The height of the image.
            width (int): The width of the image.
            cx (int): The x-coordinate of the rectangle's center.
            cy (int): The y-coordinate of the rectangle's center.
            w (int): The width of the rectangle.
            h (int): The height of the rectangle.

        Returns:
            tuple: A tuple containing:
                - AOI_mask (numpy.ndarray): A binary mask with the AOI rectangle filled.
                - Rectangle (numpy.ndarray): The same mask with the rectangle drawn in color.
        """

        AOI_mask = np.zeros((height, width), dtype=np.uint8)
        # Define rectangle parameters
        # cx, cy = (~dwell_proj.transform) *(geo_centr_x, geo_centr_y) # inverse of Affine is ~src.transorm
        cx = int(cx)
        cy = int(cy)
        # w, h = 200, 200     # Width and height of the rectangle
        # Calculate top-left and bottom-right corners
        top_left = (cx - w // 2, cy - h // 2)
        bottom_right = (cx + w // 2, cy + h // 2)

        AOI_mask = cv2.rectangle(AOI_mask, top_left, bottom_right, 1, cv2.FILLED)
        Rectangle = cv2.rectangle(AOI_mask, top_left, bottom_right, color=(255, 0, 0), thickness=2)

        return AOI_mask, Rectangle
        
    def get_rgbd_auto_AOI(self, model: mujoco.MjModel, data: mujoco.MjData):
        
        rgb, _ = self._render_camera_view(model, data, self.perception_context)
        
        # Process images
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        rgb_bgr = cv2.rotate(rgb_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE) 
        depth_gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)
        # self.search_tags(rgb_bgr)
        current_time = data.time
        if not hasattr(self, 'start_time'):
            self.start_time = current_time
            
        if current_time - self.start_time < 2.0:
            # Just display camera without tracking
            # cv2.imshow("RGB Camera View", rgb_bgr)
            return
        
        
        if not self.initialized:
            self.AOI_mask, self.Rectangle = self._define_AOI( self.width, self.height, 160, 340, 140, 180)
                    
            self.old_gray = depth_gray.copy()
                                
            try:
                self.edges = cv2.goodFeaturesToTrack(self.old_gray, mask=self.AOI_mask, **self.params_shitomasi)
            except cv2.error as e:
                print(f"[ERROR] goodFeaturesToTrack failed: {e}")
                cv2.imshow("RGB Camera View (Fallback)", rgb_bgr)
                return
        
            self.canvas = np.zeros_like(rgb_bgr)
            self.initialized = True
            return  # wait for next frame to do tracking

        try: 
            
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(self.old_gray, depth_gray, self.edges, None, **self.params_lucas_kanade)
            
            backup_points = next_points.copy() #make a backup in case of failure
            
        except cv2.error as e:
            print(f"[ERROR] Optical flow calculation failed: {e}")
            #utilize the last good points
            good_new = backup_points[status == 1]
            good_old = self.edges[status == 1]
            return 
        
        
        good_new = next_points[status == 1]
        good_old = self.edges[status == 1]

        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            
            
            self.canvas = cv2.line(self.canvas, (int(a), int(b)), (int(c), int(d)), self.colours[i].tolist(), 2)
            
            
            rgb_bgr = cv2.circle(rgb_bgr, (int(a), int(b)), 5, self.colours[i].tolist(), -1)

        result = cv2.add(rgb_bgr, self.canvas)
        
        rectangle_3ch = cv2.merge([self.Rectangle] * 3)  # Make it 3-channel, scale up to 0–255
        result = cv2.add(result, rectangle_3ch)
        
        # result = cv2.add(result, self.Rectangle)
        cv2.imshow("Optical Flow (Sparse)", result)
        cv2.waitKey(1)

        self.old_gray = depth_gray.copy()
        self.edges = good_new.reshape(-1, 1, 2)

    def search_tags(self, frame):
        # Convert frame to gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray, False, camera_params=None, tag_size = None)
        
        if tags is not None:

            # If no tags is found
            if (len(tags) ==0):
                # print("After search found = ",len(tags))
                self.NO_TAGS = True
                return 0
            
            # Return what you found
            else:
                # print("After search found = ",len(tags))
                return tags

    def init_tags(self, frame):

        # Convert frame to gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray, False, camera_params=None, tag_size = None)
        self.AllTagsDict = {}
        self.n_tags = 0
        if tags is not None:

            # Store how many tags we assign
            self.n_tags = len(tags)
            print("From init: found = ", self.n_tags)

            # If 0 tags was found
            if (self.n_tags==0):
                self.NO_TAGS = True

            # Else store the detected tags
            else:
                self.NO_TAGS = False
                # Reset tags
                # self.AllTagsDict = {}
                # Per detected tag - constr. an object
                for tag in tags:
                    self.add_tag(tag.tag_id, tag.corners, True)


    def add_tag(self, tag_id, corners, flags_to_update):
        self.AllTagsDict[tag_id] = Tag(id=tag_id, corners=corners, VISIBLE=flags_to_update)

    # Cb for Detect and Tracking
    def Cb_DnT(self, frame):
        # Render
        # rgb, _ = self._render_camera_view(model, data, self.perception_context)
        rgb = frame
        # Process images
        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        rgb_bgr1 = cv2.rotate(rgb_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE) 
        rgb_bgr2 = cv2.flip(rgb_bgr1, 1)
        # depth_gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)

        frame = rgb_bgr2
        # Wait 1.0sec to work with perception things
        # current_time = data.time
        # if not hasattr(self, 'start_time'):
        #     self.start_time = current_time
        # if current_time - self.start_time < 1.0:
        #     return

        # Init tags if needed
        if self.NO_TAGS:
            print("NO_TAGS: try to init them (if so)")
            self.init_tags(frame=frame)
        
        # The next comment regards only the initial part
        # Since at least 1 tag is found (self.NO_TAGS has became False, so not ok) -> perceive, from previous
        if not self.NO_TAGS:
            # print("Ready to search for tags:")
            # Reset flag as for now i have not seen anything
            # Control from VISIBLE flag, if those are detected again in the current frame
            for id, t in self.AllTagsDict.items():
                t.VISIBLE = False
            # Serach for tags in this current frame
            tags_res = self.search_tags(frame=frame)
            if (tags_res == 0):

                # This will terminate the loop, it forces the next Cb to init tags
                self.NO_TAGS = True #oups, no tags found, let's go again
                self.n_tags = 0
                # print("Oh I can see NO tags :( )")
                return
            else:
                # for each detected tags update corners
                for tag in tags_res:
                    if tag.tag_id in self.AllTagsDict:
                        # print("See a detected tag again: id = ", tag.tag_id)
                        self.AllTagsDict[tag.tag_id].update_corners(corners=tag.corners)
                        self.AllTagsDict[tag.tag_id].VISIBLE = True
                    else:
                        # print("New detected tag: id = ", tag.tag_id)
                        self.add_tag(tag.tag_id, tag.corners, True)

                to_remove_ids = [id for id, tag in self.AllTagsDict.items() if not tag.VISIBLE]
                for id in to_remove_ids:
                    del self.AllTagsDict[id]
                self.n_tags = len(self.AllTagsDict)
                # print("My Perception is: \n", self.AllTagsDict)
                # print("n_tags = ", self.n_tags)

            # Centroids if more than 2 tags
            if (self.n_tags < 3):
                self.area_btw_tags = 0.0
            else:
                points = []
                for id,tag in self.AllTagsDict.items():
                    points.append(tag.centroid)

                self.coords = order_clockwise(points)
                poly = Polygon(self.coords)
                self.area_btw_tags = poly.area

        annotated_frame = frame.copy()
        for id, tag in self.AllTagsDict.items():

            for idx in range(len(tag.corners)):
                cv2.line(annotated_frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 0, 255))

            # cv.circle(annotated_frame, p1, radius=5, color=(0, 0, 255), thickness=-1)  # red dot

            cv2.circle(annotated_frame, tuple(tag.corners[0].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
            cv2.circle(annotated_frame, tuple(tag.corners[1].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
            cv2.circle(annotated_frame, tuple(tag.corners[2].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
            cv2.circle(annotated_frame, tuple(tag.corners[3].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot



        # print(self.area_btw_tags)

        if not (len(self.coords) == 0): 
            pts = np.array(self.coords, dtype=np.int32)
            pts = pts.reshape((-1, 1, 2))

            cv2.polylines(annotated_frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            
        cv2.imshow('Annotated Frame', annotated_frame)

        
        # result = cv2.add(result, self.Rectangle)
        # cv2.imshow("annotated_frame", annotated_frame)
        # cv2.waitKey(1)

    # Cb for Detect and Tracking
    def Cb_DnT_realcamera(self, frame):
        # Render
        rgb = frame

        # Init tags if needed
        if self.NO_TAGS:
            print("NO_TAGS: try to init them (if so)")
            self.init_tags(frame=rgb)

        
        # The next comment regards only the initial part
        # Since at least 1 tag is found (self.NO_TAGS has became False, so not ok) -> perceive, from previous
        if not self.NO_TAGS:
            print("Ready to search for tags:")
            # Reset flag as for now i have not seen anything
            # Control from VISIBLE flag, if those are detected again in the current frame
            for id, t in self.AllTagsDict.items():
                t.VISIBLE = False
            # Serach for tags in this current frame
            tags_res = self.search_tags(frame=rgb)
            if (tags_res == 0):

                # This will terminate the loop, it forces the next Cb to init tags
                self.NO_TAGS = True #oups, no tags found, let's go again
                self.n_tags = 0
                print("Oh I can see NO tags :( )")
                return
            else:
                # for each detected tags update corners
                for tag in tags_res:
                    if tag.tag_id in self.AllTagsDict:
                        # print("See a detected tag again: id = ", tag.tag_id)
                        self.AllTagsDict[tag.tag_id].update_corners(corners=tag.corners)
                        self.AllTagsDict[tag.tag_id].VISIBLE = True
                    else:
                        # print("New detected tag: id = ", tag.tag_id)
                        self.add_tag(tag.tag_id, tag.corners, True)

                to_remove_ids = [id for id, tag in self.AllTagsDict.items() if not tag.VISIBLE]
                for id in to_remove_ids:
                    del self.AllTagsDict[id]
                self.n_tags = len(self.AllTagsDict)
                print("My Perception is: \n", self.AllTagsDict)
                print("n_tags = \n", self.n_tags)

            # Centroids if more than 2 tags
            if (self.n_tags < 3):
                self.area_btw_tags = 0.0
            else:
                points = []
                for id,tag in self.AllTagsDict.items():
                    points.append(tag.centroid)

                self.coords = order_clockwise(points)
                poly = Polygon(self.coords)
                self.area_btw_tags = poly.area

    def segment_occlusions(self, depth):

        H, W = depth.shape
        # Sobel for edges
        grad_edges = cv2.Sobel(depth, cv2.CV_64F, 1, 0, ksize=3)

        # If needed: identify the regions with strong edges - vertical
        vertical_edges = np.abs(grad_edges) > np.percentile(np.abs(grad_edges), 10)  

        # Find the closest vertical component
        closest_pixel = None
        closest_depth = np.inf

        for y in range(H):
            for x in range(W):
                if vertical_edges[y, x]:
                    depth_value = depth[y, x]
                    if depth_value < closest_depth:
                        closest_depth = depth_value
                        closest_pixel = (x, y)

        # Binary mask based on the vertical edges
        binary_mask = vertical_edges.astype(np.uint8) * 255
        # kernel = np.ones((3,3), np.uint8)  
        # cleaned_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours on the binary mask
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If contours are found, process them
        if contours:
            # Get the largest contour (this should correspond to the occlusion)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Vizualize results
            cv2.rectangle(depth, (x, y), (x + w, y + h), (0, 255, 0), 2) 
            cv2.drawContours(depth, [largest_contour], -1, (0, 255, 0), thickness=cv2.FILLED)  
            
        # Display the result
        if closest_pixel:
            x, y = closest_pixel
            cv2.circle(depth, (x, y), 5, (0, 0, 255), -1)  # Mark the closest vertical component with a red dot
        
            # # Store them
        self.contour_x1 = x
        self.contour_y1 = y
        self.contour_x2 = x + w
        self.contour_y2 = y + h

        # Show the depth image with the bounding box around the occlusion
        cv2.imshow("Segmented Occlusion", depth)
        cv2.waitKey(1)


        ######### OPTION 2 ################
        # H, W = depth.shape
        # # # Sobel for edges
        # grad_edges = cv2.Sobel(depth, cv2.CV_64F, 1, 0, ksize=3)

        # # # Identify the regions with strong vertical edges
        # vertical_edges = np.abs(grad_edges) > 10  # Set a fixed threshold instead of using percentiles

        # # Apply morphological transformations to separate objects if needed
        # kernel = np.ones((5,5), np.uint8)  # Larger kernel size for separation
        # cleaned_mask = cv2.morphologyEx(vertical_edges.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        # # Find contours on the binary mask
        # contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # # If contours are found, process them
        # if contours:
        #     for contour in contours:
        #         # Get the bounding box of each contour
        #         x, y, w, h = cv2.boundingRect(contour)

        #         # Only process contours that are large enough (filter out small noise)
        #         if cv2.contourArea(contour) > 1000:  # Adjust the minimum area threshold as needed
        #             # Draw the bounding box on the depth image
        #             cv2.rectangle(depth, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box
        #             cv2.drawContours(depth, [contour], -1, (0, 255, 0), thickness=cv2.FILLED)  # Fill the occlusion area

        # # Show the depth image with the bounding box around the occlusion
        # cv2.imshow("Segmented Occlusion", depth)
        # cv2.waitKey(1)

