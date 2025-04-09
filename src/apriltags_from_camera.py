import numpy as np
import cv2 as cv
import os
from Perception import Perception
import apriltag

# perception = Perception()

try:
    import yaml
except:
    raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')

options = apriltag.DetectorOptions(families='tag36h11',
                                 border=1,
                                 nthreads=4,
                                 quad_decimate=1.0,
                                 quad_blur=0.0,
                                 refine_edges=True,
                                 refine_decode=False,
                                 refine_pose=False,
                                 debug=False,
                                 quad_contours=True)
detector = apriltag.Detector(options)

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break


#     perception.Cb_DnT_realcamera(frame=frame)



    # Annotated frame
    annotated_frame = frame.copy()

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    detections = detector.detect(gray)
    for d in detections:
        print(d.tag_id)
        # print(d.corners)
        print(d.decision_margin)
    # for id, tag in perception.AllTagsDict.items():

    #     for idx in range(len(tag.corners)):
    #         cv.line(frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 0, 255))

    #     cv.circle(annotated_frame, tuple(tag.corners[0].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
    #     cv.circle(annotated_frame, tuple(tag.corners[1].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
    #     cv.circle(annotated_frame, tuple(tag.corners[2].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
    #     cv.circle(annotated_frame, tuple(tag.corners[3].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot

    #     print(annotated_frame.shape)
    #     print("Shape corners", tag.p1)
    #     print("Shape corners", tag.p2)
    #     print("Shape corners", tag.p3)
    #     print("Shape corners", tag.p4)

    # Display the resulting frame
    cv.imshow('Actual Camera', frame)
    cv.imshow('Annotated Frame', annotated_frame)
    if cv.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()