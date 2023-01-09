#!/usr/bin/env python2
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from my_pkg.msg import Camera
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from time import sleep
import numpy as np
#import keyboard
import rospy
import rosbag


######################################################

def getRotation(rotation):

    if float(rotation) < 15 or float(rotation) > -15:
        if float(rotation) > 0:
            rotation = 179.99 - rotation
        elif float(rotation) < 0:
            rotation = -179.99 - rotation
        else:
            rotation = 179.99 - rotation
    else:
        rotation = 179.99

    return rotation

######################################################

def getAngle(distance):

    angle=35

    angle = 0.3587*distance-25.825  

    print(angle)

    if angle > 35:

        angle = 35

    angle = (angle-5)

    return angle

#######################################################

def deg2rad(deg):
    rad = deg*np.pi/180
    return rad

#######################################################

def throwSequence(distance, rotation):

    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.set_trajectory_time(moving_time=1, accel_time=0.3)

    # Wake up
    bot.arm.set_ee_pose_components(x=0.2, z=0.2, blocking=False)
    bot.gripper.open()

    # Pick up ball
    if float(rotation) > 0:
        bot.arm.set_single_joint_position("waist", deg2rad(179.99))
    elif float(rotation) < 0:
        bot.arm.set_single_joint_position("waist", deg2rad(-179.99))
    else:
        bot.arm.set_single_joint_position("waist", deg2rad(179.99))
    
    bot.arm.set_ee_cartesian_trajectory(x=0.204, z=-0.15)
    bot.gripper.close()

    # Prepare Throw
    bot.arm.set_single_joint_position("shoulder", deg2rad(31))
    bot.arm.set_single_joint_position("elbow", deg2rad(-10))
    bot.arm.set_single_joint_position("wrist_angle", deg2rad(getAngle(float(distance))))
    bot.arm.set_single_joint_position("waist", deg2rad(getRotation(float(rotation))))           #Here we adjust the angle so we can hit all the cups
    sleep(1)

    # Throw
    bot.arm.set_single_joint_position("shoulder", deg2rad(-25), moving_time=0.33, accel_time=0.1, blocking=False)
    bot.arm.set_single_joint_position("elbow", deg2rad(-60), moving_time=0.2, accel_time=0.1, blocking=False)
    
    # Go to sleep

    sleep(1)
    bot.arm.set_trajectory_time(moving_time=1, accel_time=0.3)
    bot.arm.set_single_joint_position("waist", deg2rad(0)) 
    bot.arm.go_to_sleep_pose()

    print("Ready to fire again!")
    
#######################################################

def main():

    # Create a service proxy
    service_proxy = rospy.ServiceProxy('get_latest_message', Trigger)

    # Call the service
    response = service_proxy.call()

    # Print the response
    print('Latest message: ' + response.message)

    distance, rotation = response.message.split('/')
    throwSequence(distance, rotation)
    #keyboard.on_press_key(' ', throwSequence(distance, rotation))

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass