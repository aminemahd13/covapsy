# Copyright 1996-2022 Cyberbotics Ltd.
#
# Control of the TT-02 car for CoVAPSy simulator for Webots 2023b
# Inspired by vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Taynaud
# July 2023

from vehicle import Driver
from controller import Lidar

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
maxAngle = 0.28 # rad (strange, the car is defined for a limit of 0.31 rad...)

# Reset speed and steering
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

# Manual and auto modes disabled by default
manualMode = False
autoMode = False

print("Click on the 3D view to start")
print("m for manual mode, a for auto mode, n for stop, l to display lidar data")
print("In manual mode, use arrow keys to accelerate, brake, and steer")

while driver.step() != -1:

    speed = driver.getTargetCruisingSpeed()

    while True:
        # Lidar data acquisition
        lidar_data = lidar.getRangeImage()
        
        # Get keyboard key
        currentKey = keyboard.getKey()
        if currentKey == -1:
            break
            
        if currentKey == ord('m') or currentKey == ord('M'):
            if not manualMode:
                manualMode = True
                autoMode = False
                print("------------ Manual Mode Enabled ---------------")
        elif currentKey == ord('n') or currentKey == ord('N'):
            if manualMode or autoMode:
                manualMode = False
                autoMode = False
                print("-------- Manual and Auto Modes Disabled -------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not autoMode: 
                autoMode = True
                manualMode = False
                print("------------ Auto Mode Enabled -----------------")
        elif currentKey == ord('l') or currentKey == ord('L'):
                print("----- Lidar data in meters, clockwise with 1° step -----")
                for i in range(len(lidar_data)):
                    print(f"{lidar_data[i]:.3f}   ", end='')
                    if (i + 1) % 10 == 0:        
                       print()
                print()
      
        # Manual mode control
        if manualMode:
            if currentKey == keyboard.UP:
                speed += 0.2
            elif currentKey == keyboard.DOWN:
                speed -= 0.2
            elif currentKey == keyboard.LEFT:
                angle -= 0.04
            elif currentKey == keyboard.RIGHT:
                angle += 0.04

    if not manualMode and not autoMode:
        speed = 0
        angle = 0
        
    if autoMode:
        speed = 3 # km/h
        # Steering angle is the difference between lidar ray measurements 
        # at (-99 + 18 * 2) = -63° and (-99 + 81 * 2) = 63°
        angle = lidar_data[240] - lidar_data[120]

    # Clamp speed and angle to max values
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
        
    if angle > maxAngle:
        angle = maxAngle
    elif angle < -maxAngle:
        angle = -maxAngle

    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)