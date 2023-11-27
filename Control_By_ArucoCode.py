import cv2 as cv
from cv2 import aruco
import numpy as np
import RPi.GPIO as GPIO
import time
from control import Control

pwmValue = 0

Control1 = Control(23, 24, 14, 15, 5, 6, 17, 27)



# Initialize the ArUco marker detection
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
param_markers = aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

# Open the default camera/webcam
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH,960)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,540)
# Define the ID of the ArUco marker to follow
target_marker_id = 0  # Replace with the ID of the marker you want to follow

# Define robot movement parameters
forward_speed = 100  # Adjust the speed as needed
stop_speed = 0

# Define a flag to track whether the robot is currently moving
robot_moving = False
marker_center_x = 0
frame_center_x = 0
# Define a function to control the robot based on marker detection
def control_robot(marker_IDs):
    global robot_moving
    global marker_center_x
    global frame_center_x

    if target_marker_id in marker_IDs:
        # The target marker is detected, so move the robot forward
        Control1.move_forward(forward_speed)
        robot_moving = True
    else:
        # The target marker is not detected, so stop the robot
        Control1.stop()
        robot_moving = False
    if robot_moving:
        # Check the marker's position to decide whether to turn left or right
        marker_center_x = ((corners[1].ravel()[0] - corners[0].ravel()[0]) // 2) + corners[0].ravel()[0]
        frame_center_x = frame.shape[1] // 2
        cv.line(frame,(frame_center_x - 100, 0), (frame_center_x - 100, frame.shape[0]),(0, 255, 255), 4)
        cv.line(frame,(frame_center_x + 100, 0), (frame_center_x + 100, frame.shape[0]),(0, 255, 255), 4)
    if marker_center_x < frame_center_x - 100:
                # Marker is to the left, so turn left
        Control1.left(30)
    elif marker_center_x > frame_center_x + 100:
        # Marker is to the right, so turn right
        Control1.right(30)

        

# Iterate through frames in the live video feed
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)
    # print(frame.shape[1])
    print(frame.shape[1])
    print(frame.shape[0])
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            # print(corners.astype(np.int32))
            # Draw marker outlines and display IDs
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            print(corners)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            cv.putText(
                frame,
                f"id: {ids[0]}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )

        # Control the robot based on detected markers
        control_robot(marker_IDs)

    # If the robot was moving but the marker is no longer detected, stop it
    if robot_moving and not marker_corners:
        Control1.stop()
        robot_moving = False

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break

# Release the camera, stop the robot, and clean up GPIO
cap.release()
Control1.stop()
cv.destroyAllWindows()
