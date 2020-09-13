#some code is copied from supplied links and from blob.py
import cv2
import numpy as np
import servos
import sensors
import signal
from ThreadedWebcam import ThreadedWebcam

def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos and sensors
    robot.setSpeedsPWM(1.5, 1.5)
    sensors.lSensor.stop_ranging()
    sensors.fSensor.stop_ranging()
    sensors.rSensor.stop_ranging()
    f.close()
    sensors.GPIO.cleanup()
    exit()

f = open("blobVSdistance.txt", "w")
signal.signal(signal.SIGINT, ctrlC)

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

#PID control for rotation
k = 1
minSpeed = -0.2
maxSpeed = 0.2
error = 0
u = 0
xposition = 0
desiredx = 320 #center of 640 pixel wide window

print("Enter x coordinate of pink goal:")
pinkx = float(input())
print("Enter y coordinate of pink goal:")
pinky = float(input())
print("Enter x coordinate of green goal:")
greenx = float(input())
print("Enter y coordinate of green goal:")
greeny = float(input())
print("Enter x coordinate of blue goal:")
bluex = float(input())
print("Enter y coordinate of blue goal:")
bluey = float(input())

pinkdistance = 0
greendistance = 0
bluedistance = 0

#set search to pink
minH = 130
maxH = 180
minS = 142
maxS = 207
minV = 55
maxV = 154
# Read until pink goal is found
while pinkdistance == 0:
  # Capture frame-by-frame
    frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    keypoints = detector.detect(mask)
    frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #distance to goal
    fDistance = sensors.fSensor.get_distance()
    if keypoints != []:
        xvalue = keypoints[0].pt[0]
        error = xvalue - desiredx
        u = k * error
        if (u > 20):
            robot.setSpeedsVW(0, maxSpeed)
        elif (u < -20):
            robot.setSpeedsVW(0, minSpeed)
        else:
            pinkdistance = (fDistance/25.4) + 6
    else:
        robot.setSpeedsVW(0, 0.4);
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break

#set search to green
minH = 35
maxH = 69
minS = 187
maxS = 247
minV = 29
maxV = 106
# Read until green goal is found
while greendistance == 0:
  # Capture frame-by-frame
    frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    keypoints = detector.detect(mask)
    frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #distance to goal
    fDistance = sensors.fSensor.get_distance()
    if keypoints != []:
        xvalue = keypoints[0].pt[0]
        error = xvalue - desiredx
        u = k * error
        if (u > 20):
            robot.setSpeedsVW(0, maxSpeed)
        elif (u < -20):
            robot.setSpeedsVW(0, minSpeed)
        else:
            greendistance = (fDistance/25.4) + 6
    else:
        robot.setSpeedsVW(0, 0.4);
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
    
#set search to blue
minH = 54
maxH = 117
minS = 207
maxS = 245
minV = 74
maxV = 239
# Read until blue goal is found
while bluedistance == 0:
  # Capture frame-by-frame
    frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    keypoints = detector.detect(mask)
    frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #distance to goal
    fDistance = sensors.fSensor.get_distance()
    if keypoints != []:
        xvalue = keypoints[0].pt[0]
        error = xvalue - desiredx
        u = k * error
        if (u > 20):
            robot.setSpeedsVW(0, maxSpeed)
        elif (u < -20):
            robot.setSpeedsVW(0, minSpeed)
        else:
            bluedistance = (fDistance/25.4) + 6
    else:
        robot.setSpeedsVW(0, 0.4);
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
    
A = (-2 * pinkx) + (2 * greenx)
B = (-2 * pinky) + (2 * greeny)
C = (pinkdistance * pinkdistance) - (greendistance * greendistance) - (pinkx * pinkx) + (greenx * greenx) - (pinky * pinky) + (greeny * greeny)
D = (-2 * greenx) + (2 * bluex)
E = (-2 * greeny) + (2 * bluey)
F = (greendistance * greendistance) - (bluedistance * bluedistance) - (greenx * greenx) + (bluex * bluex) - (greeny * greeny) + (bluey * bluey)
xposition = ((C * E) - (F * B)) / ((E * A) - (B * D))
yposition = ((C * D) - (A * F)) / ((B * D) - (A * E))
print("Distance to pink goal: " + str(pinkdistance) + " inches")
print("Distance to green goal: " + str(greendistance) + " inches")
print("Distance to blue goal:" + str(bluedistance) + " inches")
print("Robot position: (" + str(xposition) + ", " + str(yposition) + ")")
robot.setSpeedsPWM(1.5, 1.5)
# When everything done, release the video capture object
cap.stop()

# Closes all the frames
cv2.destroyAllWindows()


