import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from control import Control
# control
Control1 = Control(23, 24, 14, 15, 5, 6, 17, 27)
# ultrasonic 
TRIG=21
ECHO=20
GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BCM)
# distance 
calib_data_path = "calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 20  # centimeters

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
param_markers = cv2.aruco.DetectorParameters()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,540)
# Define the ID of the ArUco marker to follow
target_marker_id = 0  # Replace with the ID of the marker you want to follow

# Define robot movement parameters
forward_speed = 100  # Adjust the speed as needed
stop_speed = 0

# Define a flag to track whether the robot is currently moving
robot_moving = False
marker_center_x = 0
frame_center_x = 0
avoidance_distance = 0
# Define a function to control the robot based on marker detection
def control_robot(marker_IDs, dis):
    global robot_moving
    global marker_center_x
    global frame_center_x
    if target_marker_id in marker_IDs:
        # The target marker is detected, so move the robot forward
        value = (int(distance) / 400) * 100
        Control1.move_forward(value)
        robot_moving = True
    else:
        # The target marker is not detected, so stop the robot
        Control1.stop()
        robot_moving = False
    if robot_moving:
        # Check the marker's position to decide whether to turn left or right
        marker_center_x = ((corners[1].ravel()[0] - corners[0].ravel()[0]) // 2) + corners[0].ravel()[0]
        frame_center_x = frame.shape[1] // 2
        cv2.line(frame,(frame_center_x - 100, 0), (frame_center_x - 100, frame.shape[0]),(0, 255, 255), 4)
        cv2.line(frame,(frame_center_x + 100, 0), (frame_center_x + 100, frame.shape[0]),(0, 255, 255), 4)

    if marker_center_x < frame_center_x - 100:
                # Marker is to the left, so turn left
        Control1.left(40)
    elif marker_center_x > frame_center_x + 100:
        # Marker is to the right, so turn right
        Control1.right(40)
def avoidance():
    global avoidance_distance
    try:
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        GPIO.output(TRIG, False)
        time.sleep(0.2)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        # Measure distance from the first sensor
        while GPIO.input(ECHO) == 0:
            pulse_start1 = time.time()
        while GPIO.input(ECHO) == 1:
            pulse_end1 = time.time()
        pulse_duration1 = pulse_end1 - pulse_start1
        avoidance_distance = pulse_duration1 * 17150
        avoidance_distance = round(avoidance_distance, 2)    
    except UnboundLocalError:
        pulse_duration1 = 0 
while True:
    avoidance()
    print(avoidance_distance)
    ret, frame = cap.read()
    if not ret:
        break
    try:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        if marker_corners:
            rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                # cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )
            if(avoidance_distance > 75.0):
                control_robot(marker_IDs, round(distance, 2))
            else:
                Control1.stop()
        if robot_moving and not marker_corners:
            Control1.stop()
            robot_moving = False
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
    except ValueError:
        continue
    
cap.release()
cv2.destroyAllWindows()
