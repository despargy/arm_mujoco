import datetime
import sys
import numpy as np
import cv2
import argparse
import time
# Parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--path", help="Path to video file", default="race.mp4")
args = vars(ap.parse_args())

cap = cv2.VideoCapture(args["path"])


        
if not cap.isOpened():
    print("[ERROR] cannot open video file")
    sys.exit()
# Play video for one second before allowing point selection
start_time = time.time()
while time.time() - start_time < 0.8:
    ret, frame = cap.read()
    if not ret:
        print("[INFO] end of file reached")
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()
    cv2.imshow('Optical Flow', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()
        

# generate initial corners of detected object
# set limit, minimum distance in pixels and quality of object corner to be tracked
parameters_shitomasi = dict(maxCorners=100, qualityLevel=0.8, minDistance=7)
# set min size of tracked object, e.g. 15x15px
parameter_lucas_kanade = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS |
                                                                      cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# create random colours for visualization for all 100 max corners for RGB channels
colours = np.random.randint(0, 255, (100, 3))

# get first video frame
ok, frame = cap.read()
if not ok:
    print("[ERROR] cannot get frame from video")
    sys.exit()
# convert to grayscale
frame_gray_init = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Allow user to select two points to define a rectangular mask
mask_points = []

def select_mask_point(event, x, y, flags, param):
    global mask_points
    if event == cv2.EVENT_LBUTTONDOWN:
        mask_points.append((x, y))
        # Provide visual feedback by drawing a small circle
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow('Select Mask', frame)

# Create a window for mask selection and set the mouse callback
cv2.namedWindow('Select Mask')
cv2.imshow('Select Mask', frame)
cv2.setMouseCallback('Select Mask', select_mask_point)

print("Please click two points on the image to define the mask region.")

# Wait until two points have been selected
while len(mask_points) < 2:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()
        
print("Mask points selected:", mask_points)

# Determine the top-left and bottom-right coordinates
(x1, y1), (x2, y2) = mask_points[:2]
# top left = min(x1,x2) min(y1,y2)
top_left = (min(x1, x2), min(y1, y2))
print("y1", y1)
print("y2", y2)
print(f"top left height {min(y1,y2)}")

bottom_right = (max(x1, x2), max(y1, y2))

# Create a mask with the same dimensions as the grayscale frame
mask = np.zeros_like(frame_gray_init)
mask[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]] = 255

#show the mask area on the image for confirmation
masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
cv2.imshow('Mask Applied', masked_frame)
cv2.waitKey(500)  # Wait a moment to show the mask
cv2.destroyWindow('Select Mask')

# Use Shi-Tomasi to detect object corners / edges from initial frame
edges = cv2.goodFeaturesToTrack(frame_gray_init, mask = mask, **parameters_shitomasi)

# create a black canvas the size of the initial frame
canvas = np.zeros_like(frame)

# Optional recording parameter
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))
video_codec = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
prefix = 'recordings/'+datetime.datetime.now().strftime("%y%m%d_%H%M%S")
basename = "object_track.mp4"
video_output = cv2.VideoWriter("_".join([prefix, basename]), video_codec, fps, (frame_width, frame_height))


hsv_canvas = np.zeros_like(frame)
# set saturation value (position 2 in HSV space) to 255
hsv_canvas[..., 1] = 255

#loop through the remaining frames of the video
#and apply algorithm to track selected objects
while True:
    # get next frame
    ok, frame = cap.read()
    if not ok:
        print("[INFO] end of file reached")
        break
    # prepare grayscale image
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # update object corners by comparing with found edges in initial frame
    update_edges, status, errors = cv2.calcOpticalFlowPyrLK(frame_gray_init, frame_gray, edges, None,
                                                         **parameter_lucas_kanade)
    if update_edges is None or status is None:
        print("[WARNING] Optical flow failed to track features. Breaking out of loop.")
        break
    # only update edges if algorithm successfully tracked
    new_edges = update_edges[status == 1]
    # to calculate directional flow we need to compare with previous position
    old_edges = edges[status == 1]

    for i, (new, old) in enumerate(zip(new_edges, old_edges)):
        a, b = new.ravel()
        c, d = old.ravel()

        # draw line between old and new corner point with random colour
        mask = cv2.line(canvas, (int(a), int(b)), (int(c), int(d)), colours[i].tolist(), 2)
        # draw circle around new position
        frame = cv2.circle(frame, (int(a), int(b)), 5, colours[i].tolist(), -1)

    result = cv2.add(frame, mask)
    # optional recording result/mask
    # video_output.write(result)
    cv2.imshow('Optical Flow (sparse)', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # overwrite initial frame with current before restarting the loop
    frame_gray_init = frame_gray.copy()
    # update to new edges before restarting the loop
    edges = new_edges.reshape(-1, 1, 2)


# while True:
#     # get next frame
#     ok, frame = cap.read()
#     if not ok:
#         print("[ERROR] reached end of file")
#         break
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     # compare initial frame with current frame
#     flow = cv2.calcOpticalFlowFarneback(frame_gray_init, frame_gray, None, 0.5, 3, 15, 3, 5, 1.1, 0)
#     # get x and y coordinates
#     magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
#     # set hue of HSV canvas (position 1)
#     hsv_canvas[..., 0] = angle*(180/(np.pi/2))
#     # set pixel intensity value (position 3
#     hsv_canvas[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)

#     frame_rgb = cv2.cvtColor(hsv_canvas, cv2.COLOR_HSV2BGR)

#     # optional recording result/mask
#     video_output.write(frame_rgb)

#     cv2.imshow('Optical Flow (dense)', frame_rgb)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

#     # set initial frame to current frame
#     frame_gray_init = frame_gray


cv2.destroyAllWindows()
cap.release()