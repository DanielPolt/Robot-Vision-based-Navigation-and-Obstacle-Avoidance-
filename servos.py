import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO
import os.path

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 1
RSERVO = 0

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

#constants
RADIUS = 1.305
CIRCUMFERENCE = 8.200
DMID = 1.975

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    
    exit()

#class with encoder functions
class Robot:
    def __init__(self, ltickcount, rtickcount):
        self.ltickcount = ltickcount
        self.rtickcount = rtickcount
        self.previouscounts = (0, 0)
    
    def resetCounts(self):
        self.ltickcount = 0
        self.rtickcount = 0
    
    def getCounts(self):
        tickTuple = (self.ltickcount, self.rtickcount)
        return tickTuple
    
    def getSpeeds(self, countvalue, timevalue):
        leftspeed = (countvalue[0] - self.previouscounts[0]) / (32 * timevalue)
        rightspeed = (countvalue[0] - self.previouscounts[0]) / (32 * timevalue)
        speedTuple = (leftspeed, rightspeed)
        self.previouscounts = (countvalue[0], countvalue[1])
        return speedTuple

    def setSpeedsPWM(self, pwmLeft, pwmRight):
        pwm.set_pwm(LSERVO, 0, math.floor(pwmLeft / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(pwmRight/ 20 * 4096))
    
    def calibrateSpeeds(self):
        self.resetCounts()
        currentpwm = 1.7
        self.leftmaxrps = 0
        self.leftmaxips = 0
        self.leftminrps = 1
        self.leftminips = 1
        self.leftmap = []
        self.rightmap = []
        self.rightmaxrps = 0
        self.rightmaxips = 0
        self.rightminrps = 1
        self.rightminips = 1
        while currentpwm >= 1.3:
            self.setSpeedsPWM(currentpwm, currentpwm)
            time.sleep(1)
            speedtuple = self.getSpeeds(self.getCounts(), 1)
            leftrps = speedtuple[0]
            rightrps = speedtuple[1]
            if leftrps > self.leftmaxrps :
                self.leftmaxrps = leftrps
                self.leftmaxips = self.leftmaxrps * CIRCUMFERENCE
            if leftrps < self.leftminrps and leftrps != 0:
                self.leftminrps = leftrps
            if rightrps > self.rightmaxrps:
                self.rightmaxrps = rightrps
                self.rightmaxips = self.rightmaxrps * CIRCUMFERENCE
            if rightrps < self.rightminrps and rightrps != 0:
                self.rightminrps = rightrps
                self.rightminips = self.rightminrps * CIRCUMFERENCE
            righttuple = rightrps, currentpwm
            lefttuple = leftrps, currentpwm
            self.leftmap.append(lefttuple)
            self.rightmap.append(righttuple)
            currentpwm = round(currentpwm - 0.005, 3)            
        f = open("calibration_data.txt", "w")
        f.write(str(self.leftmaxrps))
        f.write("\n")
        f.write(str(self.leftmaxips))
        f.write("\n")
        f.write(str(self.leftminrps))
        f.write("\n")
        f.write(str(self.leftminips))
        f.write("\n")
        f.write(str(self.rightmaxrps))
        f.write("\n")
        f.write(str(self.rightmaxips))
        f.write("\n")
        f.write(str(self.rightminrps))
        f.write("\n")
        f.write(str(self.rightminips))
        f.write("\n")
        for tuple in self.leftmap:
            f.write(str(tuple[0]))
            f.write(" ")
            f.write(str(tuple[1]))
            f.write("\n")
        for tuple in self.rightmap:
            f.write(str(tuple[0]))
            f.write(" ")
            f.write(str(tuple[1]))
            f.write("\n")
        f.close()
        
    #forward    
    def setSpeedsRPS(self, rpsLeft, rpsRight):
        negativeleft = False
        negativeright = False
        if rpsLeft < 0:
            rpsLeft = abs(rpsLeft)
            negativeleft = True
        difference = 1
        for tuple in self.leftmap:
            if abs(tuple[0] - rpsLeft) < difference and tuple[1] >= 1.5:
                pwmLeft = tuple[1]
                difference = abs(tuple[0] - rpsLeft)
            else:
                continue
        if rpsRight < 0:
            rpsRight = abs(rpsRight)
            negativeright = True
        difference = 1
        for tuple in self.rightmap:
            if abs(tuple[0] - rpsRight) < difference and tuple[1] <= 1.5:
                pwmRight = tuple[1]
                difference = abs(tuple[0] - rpsRight)
            else:
                continue
        difference = 1
        if negativeleft == True:
            for tuple in self.leftmap:
                if abs(tuple[0] - rpsLeft) < difference and tuple[1] <= 1.5:
                    pwmLeft = tuple[1]
                    difference = abs(tuple[0] - rpsLeft)
                else:
                    continue
        difference = 1
        if negativeright == True:
            for tuple in self.rightmap:
                if abs(tuple[0] - rpsRight) < difference and tuple[1] >= 1.5:
                    pwmRight = tuple[1]
                    difference = abs(tuple[0] - rpsRight)
                else:
                    continue
        self.setSpeedsPWM(pwmLeft, pwmRight)
    
    def setSpeedsIPS(self, ipsLeft, ipsRight):
        rpsLeft = ipsLeft / CIRCUMFERENCE
        rpsRight = ipsRight / CIRCUMFERENCE
        self.setSpeedsRPS(rpsLeft, rpsRight)
        
    def setSpeedsVW(self, v, w):
        circleradius = v / w
        vleft = w * (circleradius + DMID)
        vright = w * (circleradius - DMID)
        self.setSpeedsIPS(vleft, vright)
        
#functions in initEncoders inspired by encoders.py example
def initEncoders():
    
    def onLeftEncode(pin):
        robot.ltickcount += 1

    def onRightEncode(pin):
        robot.rtickcount += 1

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
    GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

robot = Robot(0,0)        
# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library, 
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

initEncoders()
if os.path.exists("calibration_data.txt"):
    with open("calibration_data.txt", "r") as f:
        filedata = []
        for line in f:
            filedata.append(line)
        robot.leftmaxrps = float(filedata[0])
        robot.leftmaxips = float(filedata[1])
        robot.leftminrps = float(filedata[2])
        robot.leftminips = float(filedata[3])
        robot.rightmaxrps = float(filedata[4])
        robot.rightmaxips = float(filedata[5])
        robot.rightminrps = float(filedata[6])
        robot.rightminips = float(filedata[7])
        lineno = 8
        robot.leftmap = []
        while lineno < 89:
            robot.leftmap.append(tuple(map(float, filedata[lineno].split(' '))))
            lineno += 1
        robot.rightmap = []
        while lineno < 170:
            robot.rightmap.append(tuple(map(float, filedata[lineno].split(' '))))
            lineno += 1
        f.close()
else:
    robot.calibrateSpeeds()
robot.setSpeedsPWM(1.5, 1.5)