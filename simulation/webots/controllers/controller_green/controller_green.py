# Copyright 1996-2022 Cyberbotics Ltd.
#
# Control of the TT-02 car for CoVAPSy simulator for Webots 2023b
# Inspired by vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Raynaud
# August 2023

from vehicle import Driver
from controller import Lidar
import numpy as np
import time

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud() 

# Keyboard
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# Speed in km/h
speed = 0
maxSpeed = 28 # km/h

# Steering angle
angle = 0
maxAngleDegrees = 16

# Reset speed and steering
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

lidar_table_mm = [0] * 360

def set_speed_m_s(speed_m_s):
    """Sets the speed in meters per second, converted to km/h."""
    speed = speed_m_s * 3.6
    if speed > maxSpeed:
        speed = maxSpeed
    if speed < 0:
        speed = 0
    driver.setCruisingSpeed(speed)
     
def set_steering_degrees(angle_degrees):
    """Sets the steering angle in degrees, converted to radians for the driver."""
    if angle_degrees > maxAngleDegrees:
        angle_degrees = maxAngleDegrees
    elif angle_degrees < -maxAngleDegrees:
        angle_degrees = -maxAngleDegrees   
    # Conversion to radians: -angle because of the coordinate system direction
    angle_rad = -angle_degrees * 3.14 / 180
    driver.setSteeringAngle(angle_rad)

def reverse(): 
    """On the real car, there is a stop then a reverse movement for 1s."""
    driver.setCruisingSpeed(-1)  

# Auto mode disabled by default
autoMode = False
print("Click on the 3D view to start")
print("a for auto mode (no manual mode on TT02_yellow), n for stop")

while driver.step() != -1:
    while True:
        # Keyboard key acquisition
        currentKey = keyboard.getKey()
 
        if currentKey == -1:
            break
       
        elif currentKey == ord('n') or currentKey == ord('N'):
            if autoMode:
                autoMode = False
                print("-------- Yellow TT-02 Auto Mode Disabled -------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not autoMode: 
                autoMode = True
                print("------------ Yellow TT-02 Auto Mode Enabled -----------------")
    
    # Lidar raw data acquisition
    raw_lidar_data = lidar.getRangeImage()
    for i in range(360):
        # Filter data between 0 and 20 meters
        if (raw_lidar_data[-i] > 0) and (raw_lidar_data[-i] < 20):
            lidar_table_mm[i - 180] = 1000 * raw_lidar_data[-i]
        else:
            lidar_table_mm[i - 180] = 0
   
    if not autoMode:
        set_steering_degrees(0)
        set_speed_m_s(0)
        
    if autoMode:
        ########################################################
        # Student Program using:
        #    - the table: lidar_table_mm
        #    - the function: set_steering_degrees(...)
        #    - the function: set_speed_m_s(...)
        #    - the function: reverse()
        #######################################################
   
        # Steering logic: simplified calculation based on specific lidar angles
        # Sector of 20° each, 10 sectors total numbered from 0 to 9    
        angle_degrees = 0.02 * (lidar_table_mm[60] - lidar_table_mm[-60])
        set_steering_degrees(angle_degrees)
        
        speed_m_s = 0.5
        set_speed_m_s(speed_m_s)
 
    #########################################################