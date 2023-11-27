import cv2 as cv
from cv2 import aruco
import numpy as np
import RPi.GPIO as GPIO
import time
from new_control import Control

pwmValue = 0

Control1 = Control(23, 24, 14, 15, 5, 6, 17, 27)

calib_data_path = "/home/w22t10/Desktop/cv/calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 20  # centimeters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

param_markers = aruco.DetectorParameters()

# Open the default camera/webcam
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH,960)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,540)
# Define the ID of the ArUco marker to follow
target_marker_id = 0  # Replace with the ID of the marker you want to follow

# Define robot movement parameters
forward_speed = 40  # Adjust the speed as needed
stop_speed = 0

# Define a flag to track whether the robot is currently moving
robot_moving = False
marker_center_x = 0
frame_center_x = 0
# Define a function to control the robot based on marker detection
def control_robot(marker_IDs, corners, distance):
    global robot_moving
    global marker_center_x
    global frame_center_x
    if target_marker_id in marker_IDs:
        # The target marker is detected, so move the robot forward
        value = (int(distance) / 400) * 100
        if(value < 100):
            Control1.move_forward(85)
            robot_moving = True
        else:
            Control1.move_forward(85)
            robot_moving = True
    else:
        # The target marker is not detected, so stop the robot
        Control1.stop()
        robot_moving = False
    if robot_moving:
        # Check the marker's position to decide whether to turn left or right
        marker_center_x = ((corners[1].ravel()[0] - corners[0].ravel()[0]) // 2) + corners[0].ravel()[0]
        frame_center_x = frame.shape[1] // 2

    if marker_center_x < frame_center_x - 100:
                # Marker is to the left, so turn left
        Control1.smooth_right(100)
    elif marker_center_x > frame_center_x + 100:
        # Marker is to the right, so turn right
        Control1.smooth_left(100)


while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
            # so I have rectified that mistake, I have test that out it increase the accuracy overall.
            # Calculating the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )
            print(str(round(distance, 2)))
            # cv.putText(
            #     frame,
            #     f"id: {ids[0]} Dist: {round(distance, 2)}",
            #     top_right,
            #     cv.FONT_HERSHEY_PLAIN,
            #     1.3,
            #     (0, 0, 255),
            #     2,
            #     cv.LINE_AA,
            # )
            # cv.putText(
            #     frame,
            #     f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
            #     bottom_right,
            #     cv.FONT_HERSHEY_PLAIN,
            #     1.0,
            #     (0, 0, 255),
            #     2,
            #     cv.LINE_AA,
            # )
            # print(ids, "  ", corners)
            # print(" corners "+ corners)
        # Control the robot based on detected markers
        # print(" marker_IDs "+ marker_IDs )
        control_robot(marker_IDs, corners, int(round(distance, 2)))
    if robot_moving and not marker_corners:
        Control1.stop()
        robot_moving = False  
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
    
cap.release()
cv.destroyAllWindows()
