# Usage
# python scripts/optical_flow_sparse_manual.py -p 'resources/car_race_01.mp4'
import sys
import time
import numpy as np
import cv2
import argparse

cap = cv2.VideoCapture('race.mp4')
if not cap.isOpened():
    print("[ERROR] cannot open video file")
    sys.exit()

# Play video for one second before allowing point selection
start_time = time.time()
while time.time() - start_time < 1.1:
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

# Use the frame from the end of the one second delay as the initial frame for tracking
frame_gray_init = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# set min size of tracked object, e.g. 15x15px
parameter_lucas_kanade = dict(
    winSize=(25, 25),
    maxLevel=4,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)

# define function to manually select object to track
def select_point(event, x, y, flags, params):
    global point, selected_point, old_points
    # record coordinates of mouse click
    if event == cv2.EVENT_LBUTTONDOWN:
        point = (x, y)
        selected_point = True
        old_points = np.array([[x, y]], dtype=np.float32)

# associate select function with window
cv2.namedWindow('Optical Flow')
cv2.setMouseCallback('Optical Flow', select_point)

# initialize variables updated by the callback function
selected_point = False
point = ()
old_points = ([[]])

# create a black canvas the size of the current frame
canvas = np.zeros_like(frame)

# Pause on the current frame to allow selection
cv2.imshow('Optical Flow', frame)
print("Click on the video to select a point to track, or press 'q' to quit.")
while not selected_point:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        sys.exit()

# loop through the remaining frames and track the selected point
while True:
    ret, frame = cap.read()
    if not ret:
        print("[INFO] end of file reached")
        break
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if selected_point:
        cv2.circle(frame, point, 5, (0, 0, 255), 2)
        new_points, status, errors = cv2.calcOpticalFlowPyrLK(
            frame_gray_init, frame_gray, old_points, None, **parameter_lucas_kanade
        )

        frame_gray_init = frame_gray.copy()
        old_points = new_points

        x, y = new_points.ravel()
        j, k = old_points.ravel()

        canvas = cv2.line(canvas, (int(x), int(y)), (int(j), int(k)), (0, 255, 0), 3)
        frame = cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)

    result = cv2.add(frame, canvas)
    cv2.imshow('Optical Flow', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sys.exit()