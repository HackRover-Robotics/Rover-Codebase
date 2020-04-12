from __future__ import division
import RPi.GPIO as GPIO
import time
import sys
import math
import struct
import rospy
import serial
from py_pkg import arm_functions

#from controller_input.py import getInputs
#from unicurses import *

rf = 18  #pin12
rb = 23  #pin16

lf = 4  #pin 7
lb = 17  #pin 11

frf = 24  #18
frb = 25  #22

flf = 27  #13
flb = 22  #15

brf = 20  #rightForward
brb = 21  #rightBackwards

blf = 19  #rightForward
blb = 26  #rightBackwards

waist = 1500
shoulder = 1500
elbow = 1500
wrist = 1500
claw_rot = 1500
claw_grip = 1500

GPIO.setmode(GPIO.BCM)

GPIO.setup(rf,GPIO.OUT)
GPIO.setup(rb,GPIO.OUT)
GPIO.setup(lf,GPIO.OUT)
GPIO.setup(lb,GPIO.OUT)

GPIO.output(rf,GPIO.LOW)
GPIO.output(rb,GPIO.LOW)
GPIO.output(lf,GPIO.LOW)
GPIO.output(lb,GPIO.LOW)

rightForward = GPIO.PWM(rf, 1000)
rightBackward = GPIO.PWM(rb, 1000)
leftForward = GPIO.PWM(lf, 1000)
leftBackward = GPIO.PWM(lb, 1000)


GPIO.setup(frf,GPIO.OUT)
GPIO.setup(frb,GPIO.OUT)
GPIO.setup(flf,GPIO.OUT)
GPIO.setup(flb,GPIO.OUT)
GPIO.setup(brf,GPIO.OUT)
GPIO.setup(brb,GPIO.OUT)
GPIO.setup(blf,GPIO.OUT)
GPIO.setup(blb,GPIO.OUT)

GPIO.output(frf,GPIO.LOW)
GPIO.output(frb,GPIO.LOW)
GPIO.output(flf,GPIO.LOW)
GPIO.output(flb,GPIO.LOW)
GPIO.output(brf,GPIO.LOW)
GPIO.output(brb,GPIO.LOW)
GPIO.output(blf,GPIO.LOW)
GPIO.output(blb,GPIO.LOW)

frontRF = GPIO.PWM(frf, 1000)
frontRB = GPIO.PWM(frb, 1000)
frontLF = GPIO.PWM(flf, 1000)
frontLB = GPIO.PWM(flb, 1000)
backRF = GPIO.PWM(brf, 1000)
backRB = GPIO.PWM(brb, 1000)
backLF = GPIO.PWM(blf, 1000)
backLB = GPIO.PWM(blb, 1000)

#noseRightForward.stop()
#noseRightBackward.stop()
#noseLeftForward.stop()
#noseLeftBackward.stop()
duty = 100
dutydut = 100;
noseDuty = 100;
#9C:AA:1B:97:FE:50
leftDuty = 0
rightDuty = 0
def update_stair_treads(LBTrig, LTTrig, RBTrig, RTTrig):
    if((LBTrig == 0 and LTTrig == 0) or (LBTrig != 0 and LTTrig != 0 )):
        frontLF.stop()
        frontLB.stop()
    else:
        if(LBTrig != 0):
            frontLF.stop()
            frontLB.start(LBTrig)
        else:           
            frontLB.stop()
            frontLF.start(100)
        
        
    if((RBTrig == 0 and RTTrig == 0) or (RBTrig != 0 and RTTrig != 0 )):
        frontRF.stop()
        frontRB.stop()
    else:
        if(RBTrig != 0):
            frontRF.stop()
            frontRB.start(RBTrig)
        else:           
            frontRB.stop()
            frontRF.start(100)

def leftMain(speed):
    print(speed)
    if speed > 0:
        leftForward.start(speed)
    else:
        leftBackward.start(-speed)
def rightMain(speed):
    print(speed)
    if speed > 0:
        rightForward.start(speed)
    else:
        rightBackward.start(-speed)


def testMouseInput(diffX, diffY):
    
    #currMousePos = mouse.position()
    #diffY = -(currMousePos[1] - mouseHomeY)
    #diffX = currMousePos[0] - mouseHomeX\
    diffX = int(diffX)
    diffY = int(diffY)
    if abs(diffX) < 15 and abs(diffY) < 15:
        leftForward.stop()
        rightForward.stop()
        leftBackward.stop()
        rightBackward.stop()
    else:  
        hypot = int(math.sqrt(diffY**2 + diffX**2))
        potentialSpeed = min(100, hypot)
        leftDrive = 0
        rightDrive = 0
            
        if (diffX ^ diffY) < 0:
            leftDrive = potentialSpeed
            if diffY < 0:
                leftDrive *= -1
                rightDrive = math.asin(((diffY / hypot) + 0.5) * 2) * potentialSpeed * (2/math.pi)
            else:
                rightDrive = math.asin(((diffY / hypot) - 0.5) * 2) * potentialSpeed * (2/math.pi)     
            
            mouseMoving = True
            leftMain(leftDrive)
            rightMain(rightDrive)
            
            
        elif (diffX ^ diffY) > 0:
            rightDrive =  potentialSpeed
            if diffY < 0:
                rightDrive *= -1
                leftDrive = math.asin(((diffY / hypot) + 0.5) * 2) * potentialSpeed * (2/math.pi)
            else:
                leftDrive = math.asin(((diffY / hypot) - 0.5) * 2) * potentialSpeed * (2/math.pi)
            
            mouseMoving = True
            leftMain(leftDrive)
            rightMain(rightDrive)
        else:
            mouseMoving = False
    
    
def update_main_treads(left_speed, right_speed):
    print(left_speed, right_speed)
    if abs(left_speed) < 15:
        leftForward.stop()
        leftBackward.stop()
    elif left_speed > 0:
        leftBackward.stop()
        leftForward.start(left_speed)
    else:
        leftForward.stop()
        leftBackward.start(-left_speed)
        
    if abs(right_speed) < 15:
        rightForward.stop()
        rightBackward.stop()
    elif right_speed > 0:
        rightBackward.stop()
        rightForward.start(right_speed)
    else:
        rightForward.stop()
        rightBackward.start(-right_speed)
    leftForward.start(100)
    return [left_speed, right_speed]   
        

def update_arm(input_list):
    port =serial.Serial(
    "/dev/ttyUSB0",
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    writeTimeout = 0,
    timeout = 10,
    rtscts=False,
    dsrdtr=False,
    xonxoff=False)
    
    arm_values = arm_functions.update_arm_vals(input_list)
    
    command0 = b'#0P' + (str(arm_values[0])).encode('utf-8') + b'S750'
    command1 = b'#1P' + (str(arm_values[1])).encode('utf-8') + b'S750'
    command2 = b'#2P' + (str(arm_values[2])).encode('utf-8') + b'S750'
    command3 = b'#3P' + (str(arm_values[3])).encode('utf-8') + b'S750'
    command4 = b'#5P' + (str(arm_values[4])).encode('utf-8') + b'S750'
    command5 = b'#4P' + (str(arm_values[5])).encode('utf-8') + b'S750\r'
    
    #rospy.loginfo('%d control_functions', arm_values[0])
    port.write(command0 + command1 + command2 + command3 + command4 + command5)
    port.close()
    
    
    
def update_rover_state(input_list):
    
    #update_main_treads((-input_list[16] / 32767) * 100, (-input_list[18] / 32767) * 100)
    testMouseInput((-input_list[15] / 32767) * 100, (-input_list[16] / 32767) * 100)
    #update_stair_treads(((input_list[20]+32767)/65534)*100, input_list[6], ((input_list[19]+32767)/65534)*100, input_list[7])
    update_arm(input_list)
    
    #rospy.loginfo(str(((input_list[20]+32767)/65534)*100))
    #rospy.loginfo(str(input_list[17]))
    #rospy.loginfo(str(((input_list[19]+32767)/65534)*100))
    #rospy.loginfo(str(input_list[7]))
    #rospy.loginfo('================================')
