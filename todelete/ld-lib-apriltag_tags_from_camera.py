import numpy as np
import cv2 as cv
from dt_apriltags import Detector
import os
from Perception import Perception

perception = Perception()

try:
    import yaml
except:
    raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')


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


    perception.Cb_DnT_realcamera(frame=frame)

    # Annotated frame
    annotated_frame = frame.copy()

    for id, tag in perception.AllTagsDict.items():

        for idx in range(len(tag.corners)):
            cv.line(frame, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 0, 255))

        cv.circle(annotated_frame, tuple(tag.corners[0].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
        cv.circle(annotated_frame, tuple(tag.corners[1].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
        cv.circle(annotated_frame, tuple(tag.corners[2].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot
        cv.circle(annotated_frame, tuple(tag.corners[3].astype(int)), radius=5, color=(0, 0, 255), thickness=-1)  # red dot

        # print(annotated_frame.shape)
        print("Shape corners", tag.p1)
        print("Shape corners", tag.p2)
        print("Shape corners", tag.p3)
        print("Shape corners", tag.p4)
        # print("Confidence = ",tag.confidence)
        print("Centroid = ",tag.centroid)


        cv.circle(annotated_frame, tuple(tag.centroid), radius=5, color=(0, 255, 0), thickness=-1)  # red dot

    print(perception.area_btw_tags)

    if not (len(perception.coords) == 0): 
        pts = np.array(perception.coords, dtype=np.int32)
        pts = pts.reshape((-1, 1, 2))

        cv.polylines(annotated_frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        # cv.fillPoly(annotated_frame, pts, color=(0, 100, 255))  # Transparent effect not supported here


    if (perception.n_tags == 0):
        annotated_frame = frame.copy()
        print("Believe:", perception.AllTagsDict)

    # Display the resulting frame
    cv.imshow('Actual Camera', frame)
    cv.imshow('Annotated Frame', annotated_frame)
    if cv.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()