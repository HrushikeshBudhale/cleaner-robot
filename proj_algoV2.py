# -*- coding: utf-8 -*-
"""
Created on Sat Jan 5 16:40:46 2019

@author: HRUSHIKESH
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import vrep
from operator import sub
import math

'''
#####################################################################################################################
#    Function Definitions
#####################################################################################################################
'''

def get_angle(p0, p1, p2, p3):
    ''' returns anticlockwise angle between 2 vectors
        output type = int
        output limit = [0:359]
    '''
    v0 = list( map(sub, p1, p0) )
    v1 = list( map(sub, p3, p2) )

    angle = np.degrees(np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1)))
    angle = int(360 - angle)
    if angle > 359:
        angle = angle - 360
    return angle
  

def get_centers(img):
    ''' returns center points of aruco and 0 degree line
        output type = list[[center_x, center_y], [line_x, line_y]]
    '''
    corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters = parameters)
    if len(corners) > 0:    # if aruco marker found
        center_x = 0
        center_y = 0
        line_x = 0
        line_y = 0

        for point in range(len(corners[0][0])):  
            center_x = center_x + corners[0][0][point][0]  
            center_y = center_y + corners[0][0][point][1]
            if(point == 1):
                line_x = int(center_x/2)
                line_y = int(center_y/2)
        center_x = int(center_x/4)
        center_y = int(center_y/4)
        return [center_x, center_y], [line_x, line_y]
    else:
        return None


def get_pos(imgCenter, center, line):
    ''' returns position of robot relative to aruco marker
        values are in meter
        output type = list[x, y]
    '''
    ang = get_angle(center, imgCenter, center, line)
    dist = dist_fnc(center, imgCenter)
    xp = dist * math.cos(np.deg2rad(ang)) * scale
    yp = dist * math.sin(np.deg2rad(ang)) * scale
    return [xp, yp]


def set_velocities(leftWheelVel, rightWheelVel):
    ''' sets velocities for left and right wheel of robot
    '''
    if(leftWheelVel > max_velocity): leftWheelVel = max_velocity
    if(leftWheelVel < -max_velocity): leftWheelVel = -max_velocity
    if(rightWheelVel > max_velocity): rightWheelVel = max_velocity
    if(rightWheelVel < -max_velocity): rightWheelVel = -max_velocity
    
    vrep.simxSetJointTargetVelocity(clientID, leftjoint_handle, leftWheelVel,
                                    vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, rightjoint_handle, rightWheelVel,
                                    vrep.simx_opmode_oneshot)


def get_img():
    ''' returns image from vision sensor
        output type = numpy array
        Returns 'None' if not captured
    '''
    returnCode, resolution, img = vrep.simxGetVisionSensorImage(clientID, visionSensor, 0, vrep.simx_opmode_buffer)
    if returnCode == vrep.simx_return_ok:
        img = np.array(img, dtype=np.uint8)
        img.resize([resolution[1],resolution[0],3])
        img = cv2.flip(img,0)
        return img
    else:
        return None
        

def get_phi(pt, pos, ori):
    ''' returns difference between required orientation and current orientation 
        Output limits : [-180:180]
    '''
    a = get_angle(pos, pt, pos, [pos[0]+2, pos[1]])
    phi = a - ori
    if(phi > 180):
        phi = phi - 360
    elif(phi < -180):
        phi = 360 + phi
    return phi


def show_img(img, center, line):
    ''' dispays image in new window
    '''
    cv2.circle(img,(center[0],center[1]),5,red_color,-1)
    cv2.line(img,(center[0],center[1]),(line[0],line[1]),blue_color,3)
    cv2.imshow('image',img)


def dist_fnc(p1, p2):
    ''' returns Eucledian distance between 2 2D points
    '''
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)


def moveTo(pt):
    ''' sets velocity of bot considering PI control
        returns distance between bot position and destination (meter)
        returns 'None' if aruco or image not found
    '''
    new_img = get_img()
    if new_img is not None:
        img = new_img
        ans = get_centers(img)
        if ans is not None:
            center, line = ans
            ori = get_angle(imgCenter, downPt, center, line)
            pos = get_pos(imgCenter, center, line)
            # calculate angle and distance of desitnation from bot
            phi = get_phi(pt, pos, ori)
            dist = dist_fnc(pos, pt)
            
            # uncomment following to see live image
            show_img(img, center, line)
            
            v_des = 0.01*dist + 0.01*sum(vacc)  # proportional and integral control
            w_des = 0.01*phi                    # proportional control
            v_r = v_des + (wheel_separation/2)*w_des
            v_l = v_des - (wheel_separation/2)*w_des
            # calc wheel speed
            w_r = v_r/wheel_radius
            w_l = v_l/wheel_radius
            # set wheel speed
            set_velocities(w_l, w_r)
            print("velocities: ","%.2f"%w_l, "%.2f"%w_r, "dist:", "%.2f"%dist)
            
            # accumulator for integral velocity control
            vacc.append(dist)
            if len(vacc) > 5:
                vacc.pop()            

            return dist        
        else:
            return None
    else:
        return None

            
def get_next_pt(mode):
    ''' yields generator for turning points in pattern
        input: mode
                0: zig-zag    1: clockwise rectangular
    '''
    if mode == 0 :
        toggle = True
        for x in np.linspace(llp[0], urp[0], int((abs(urp[0]- llp[0]) + width)/width)):
            if toggle:
                toggle = False
                yield [x, llp[1]]
                yield [x, urp[1]]
                
            else:
                toggle = True
                yield [x, urp[1]]
                yield [x, llp[1]]
    else:
        left  = llp[0]
        up    = urp[1]
        right = urp[0]
        down  = llp[1]
        cnt = 0
        while True:
            left = llp[0] - math.copysign(cnt,left)
            yield [left, down]
            up = urp[1] - math.copysign(cnt,up)
            yield [left, up]
            right = urp[0] - math.copysign(cnt,right)
            yield [right, up]
            down = llp[1] - math.copysign(cnt,down)
            yield [right, down]
            
            if (up - down) < width or (right - left) < width:
                break
            else:
                cnt = cnt + width



def get_middle_pt(mode):
    ''' yields generator for points between endpoints
    '''
    cp = [0,0]
    for i, pt in enumerate(get_next_pt(mode)):
        if i == 0:
            cp = pt
            continue
        px = np.linspace(cp[0],pt[0],int(dist_fnc(cp,pt)/sep), endpoint=False) # generate 5 ponints from cp to pt
        py = np.linspace(cp[1],pt[1],int(dist_fnc(cp,pt)/sep), endpoint=False)
        for p in zip(px, py):
            yield list(p)
        cp = pt
    if mode == 0 :
        yield list(urp)
    else:
        yield [0,0]


'''
#####################################################################################################################
#    Simulation Setup
#####################################################################################################################
'''
# stop simulation if already running
vrep.simxFinish(-1)

# connect to remote api server
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print("connected to remote api server")
else:
    print('connection not successful')
    sys.exit("could not connect")

# start simulation
returnCode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
if returnCode == vrep.simx_return_ok:
    print('simulation Started')

# get handles for different objects in scene
returnCode, visionSensor = vrep.simxGetObjectHandle(clientID, 'vs', vrep.simx_opmode_oneshot_wait)
returnCode, leftjoint_handle = vrep.simxGetObjectHandle(clientID, 'left_joint', vrep.simx_opmode_oneshot_wait)
returnCode, rightjoint_handle = vrep.simxGetObjectHandle(clientID, 'right_joint', vrep.simx_opmode_oneshot_wait)
# start streaming images from vision sensor
returnCode, resolution, img = vrep.simxGetVisionSensorImage(clientID, visionSensor, 0, vrep.simx_opmode_streaming)

'''
#####################################################################################################################
#    Global Constants
#####################################################################################################################
'''
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters_create()

blue_color = [255,0,0]  # blue line color
red_color = [0,0,255]   # red center point
imgCenter = [512, 512]
downPt = [512, 712]
scale = 2/444   # convert pixel distance to real distance

# sets which mode to use while moving
mode = 1        # 0: zig-zag    1: clockwise rectangular

# points should be between [-2,-2] to [2,2]
llp = [-1,-1]   # lower left point
urp = [1,1]     # upper right point

width = 0.5     # distance between 2 sweeps
sep = width/2   # approx seperation between 2 checkpoints
wheel_separation = 0.208
wheel_diameter = 0.0701
wheel_radius = wheel_diameter/2
max_velocity = 3
thresh = 0.1    # distance from which point is considered visited

# accumulator for integral control of velocity
vacc = [0]  # variable
all_points = []
'''
#####################################################################################################################
#    Main Code
#####################################################################################################################
'''
# Uncomment following to set llp and urp at runtime
#llp[0], llp[1] = map(float, input("Enter lower left point: ").split())
#urp[0], urp[1] = map(float, input("Enter upper right point: ").split())

try:
    for point in get_middle_pt(mode): 
        all_points.append(point)
    total = len(all_points)
    for i, pt in enumerate(all_points):  # for every point on the path
        print("Work in progress... ", int((i/total)*100), "% Completed")
#        dist = thresh
        while(True):    # while bot not reached that point
            dist = moveTo(pt)
            if dist is not None:
                if dist < thresh:   # if bot reached that point
                    break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
                sys.exit(0)
    print("Work Done!!! 100% Completed")
    set_velocities(0, 0)    # stop bot
    cv2.destroyAllWindows() # close image windows if any open
    vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot) # pause simulation
    
except KeyboardInterrupt:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
    