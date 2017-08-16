# -*- coding: utf-8 -*-
"""
Created on Tue Jan 06 22:00:39 2015

@author: Karan Vivek Bhargava
"""
#Import Libraries:
import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting
from msvcrt import getch   #used to get keystrokes
from tqdm import tqdm
import cv2
from PIL import Image
import array

# Model for the car with two variables throttle and steering
class CarControl():
    def __init__(self, clientID, printFlag = False):
        self.clientID = clientID;
        #retrieve motor  handles
        errorCode, self.steer_handle = vrep.simxGetObjectHandle(self.clientID, 'steer_joint', vrep.simx_opmode_oneshot_wait);
        errorCode, self.motor_handle = vrep.simxGetObjectHandle(self.clientID, 'motor_joint', vrep.simx_opmode_oneshot_wait);
        errorCode, self.fl_brake_handle = vrep.simxGetObjectHandle(self.clientID, 'fl_brake_joint', vrep.simx_opmode_oneshot_wait);
        errorCode, self.fr_brake_handle = vrep.simxGetObjectHandle(self.clientID, 'fr_brake_joint', vrep.simx_opmode_oneshot_wait);
        errorCode, self.bl_brake_handle = vrep.simxGetObjectHandle(self.clientID, 'bl_brake_joint', vrep.simx_opmode_oneshot_wait);
        errorCode, self.br_brake_handle = vrep.simxGetObjectHandle(self.clientID, 'br_brake_joint', vrep.simx_opmode_oneshot_wait);
        errorCode, self.camera_f_handle = vrep.simxGetObjectHandle(self.clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait);
        vrep.simxGetVisionSensorImage(self.clientID, self.camera_f_handle, 0, vrep.simx_opmode_streaming)
        print('Received Handles...');

        self.factor = 30/(2.68*3.6);
        self.max_throttle = 19; # Kmph
        self.max_reverse_throttle = -19; #Kmph
        self.max_steer = 30; # Degrees

        self.printFlag = printFlag;

    def set_throttle(self, target_speed):
        if(target_speed > self.max_throttle):
            target_speed = self.max_throttle;
        elif(target_speed < self.max_reverse_throttle):
            target_speed = self.max_reverse_throttle;
        if(self.printFlag):
            print('Setting throttle to', target_speed);
        speed = target_speed * self.factor;
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.motor_handle, speed, vrep.simx_opmode_streaming);

    def set_steering(self, steer_pos):
        if(abs(steer_pos) > self.max_steer):
            if(steer_pos > 0):
                steer_pos = self.max_steer;
            else:
                steer_pos = -self.max_steer;
        if(self.printFlag):
            print('Setting steering to', steer_pos);
        # Convert to radians
        steer_pos = np.deg2rad(steer_pos);        
        errorCode = vrep.simxSetJointTargetPosition(self.clientID, self.steer_handle, steer_pos, vrep.simx_opmode_streaming);

    def get_info(self):
        # Check velocity
        err, bl_wheel_vel = vrep.simxGetObjectFloatParameter(self.clientID, self.bl_brake_handle, vrep.sim_jointfloatparam_velocity, vrep.simx_opmode_streaming);
        err, br_wheel_vel = vrep.simxGetObjectFloatParameter(self.clientID, self.br_brake_handle, vrep.sim_jointfloatparam_velocity, vrep.simx_opmode_streaming);
        rear_wheel_velocity = ((bl_wheel_vel) + (br_wheel_vel))/2.0;
        linear_velocity = rear_wheel_velocity * 0.09 * 3.6; # Kmph

        throttle = linear_velocity;
        steer_errorCode, steer_pos = vrep.simxGetJointPosition(self.clientID, self.steer_handle, vrep.simx_opmode_streaming);
        if(self.printFlag):
            print('Throttle:', throttle, 'Steering:', steer_pos);

    def get_image(self):
        err, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.camera_f_handle, 0, vrep.simx_opmode_buffer);
        if err == vrep.simx_return_ok:
            print(image);
            image_byte_array = array.array('b', image);
            # print(image_byte_array);
            image_buffer = Image.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "BGR", 0, 1);
            img_out = np.asarray(image_buffer);
            img_out = cv2.flip(img_out, 0);
            return 1, img_out;

        elif err == vrep.simx_return_novalue_flag:
            return 0,None;
        else:
            return err,None;

vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Get the client ID

if clientID!=-1:  #check if client connection successful
    print('Connected to remote API server')
    
else:
    print('Connection not successful')
    sys.exit('Could not connect')

# Initialize car control object
car = CarControl(clientID, printFlag = False);

for i in tqdm(range(5)):
    time.sleep(0.1);
    car.set_steering(20); # Degrees
    car.set_throttle(100); #Kmph
    err, img = car.get_image();
    