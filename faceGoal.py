#some code is copied from supplied links and from blob.py
import cv2
import numpy as np
import servos
from ThreadedWebcam import ThreadedWebcam

robot = servos.robot
# params very similar to params.yaml from sample program
params = cv2.SimpleBlobDetector_Params()
params.thresholdStep = 1
params.minThreshold = 127
params.maxThreshold = 128
params.minRepeatability = 0
params.minDistBetweenBlobs = 10
params.filterByColor = 1
params.blobColor = 255
params.filterByArea = 1
params.minArea = 5000
params.maxArea = 300000
params.filterByCircularity = 0
params.filterByInertia = 0
params.filterByConvexity = 0
detector = cv2.SimpleBlobDetector_create(params)
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = ThreadedWebcam()
cap.start()

#PID control
k = 1
minSpeed = -0.2
maxSpeed = 0.2
error = 0
u = 0
xposition = 0
desiredx = 320 #center of 640 pixel wide window

#prompt the user to enter color of the goal
#values will vary based on lighting
print("Enter color of goal:")
color = input()
if color == "pink":
    minH = 139
    maxH = 180
    minS = 128
    maxS = 208
    minV = 64
    maxV = 228
elif color == "green":
    minH = 22
    maxH = 50
    minS = 180
    maxS = 255
    minV = 17
    maxV = 105
else: #blue
    minH = 84
    maxH = 119
    minS = 170
    maxS = 241
    minV = 44
    maxV = 255
# Read until video is completed
while True:
  # Capture frame-by-frame
    frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    keypoints = detector.detect(mask)
    frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
# Display the resulting frame (during testing)
    #cv2.imshow('Frame', mask)
    #cv2.imshow('Frame2', frame_with_keypoints)

    if keypoints != []:
        xvalue = keypoints[0].pt[0]
        error = xvalue - desiredx
        u = k * error
        if (u > 20):
            robot.setSpeedsVW(0, maxSpeed)
        elif (u < -20):
            robot.setSpeedsVW(0, minSpeed)
        else:
            robot.setSpeedsPWM(1.5, 1.5)
    else:
        robot.setSpeedsVW(0, 0.4);
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break


robot.setSpeedsPWM(1.5, 1.5)
# When everything done, release the video capture object
cap.stop()

# Closes all the frames
cv2.destroyAllWindows()
