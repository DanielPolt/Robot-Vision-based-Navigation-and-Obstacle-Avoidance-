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

#PID control for movement
speed = 0
kMove = 1
minSpeedMove = -2
maxSpeedMove = 2
errorMove = 0
uMove = 0
desiredDistance = 254 #10 inches to edge of cylinde, 14 to goal
distance = 0

#prompt the user to enter color of the goal
print("Enter color of goal:")
color = input()
if color == "pink":
    minH = 130
    maxH = 180
    minS = 142
    maxS = 207
    minV = 55
    maxV = 154
elif color == "green":
    minH = 35
    maxH = 69
    minS = 187
    maxS = 247
    minV = 29
    maxV = 106
else: #blue
    minH = 84
    maxH = 119
    minS = 95
    maxS = 161
    minV = 0
    maxV = 51

# Read until video is completed
while True:
  # Capture frame-by-frame
    frame = cap.read()
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    keypoints = detector.detect(mask)
    frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #distance to goal
    fDistance = sensors.fSensor.get_distance()
# Display the resulting frame
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
            distance = fDistance
            errorMove = (-1 * desiredDistance) + distance
            uMove = kMove * errorMove
            if(uMove > (maxSpeedMove / 0.0393701)):
                speed = maxSpeedMove
            elif(uMove < (minSpeedMove / 0.0393701)):
                speed  = minSpeedMove
            else:
                speed = uMove * 0.0393701
        
            if (abs(speed) < robot.rightminips):
                robot.setSpeedsPWM(1.5, 1.5)
            else:
                robot.setSpeedsIPS(speed, speed)
        f.write(str(fDistance))
        f.write(",")
        f.write(str(keypoints[0].size))
        f.write('\n')
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

