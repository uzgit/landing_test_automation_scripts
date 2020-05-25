#!/usr/bin/env python3

import signal
import sys
import os
import time

import rospy
import mavros
from mavros import command
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import State
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
'''
coordinates = []
flight_plan = open("/home/joshua/Documents/flight_plans/coordinates")
for line in flight_plan:
    info = line.rstrip().split(",")
    coordinates.append((info[0], info[1], info[2]))
'''
armed = False
latitude  = 0
longitude = 0
altitude  = 0

def sigint_handler(sig, frame):

    print("Quitting...")
    sys.exit(0)

def position_callback(data):

    global latitude
    global longitude
    global altitude

    latitude  = data.latitude
    longitude = data.longitude
    altitude  = data.altitude

#    print("(%s, %s, %s)" % (latitude, longitude, altitude))

def state_callback(data):
    
    global armed
    armed = data.armed

#    print("Armed: %s" % armed)

def enable_landing():
    enable_landing_command = "rosrun mavros mavcmd long 183 11 1900 0 0 0 0 0"
    print("Enabling landing via command '%s'..." % enable_landing_command)
    os.system(enable_landing_command)

def disable_landing():
    disable_landing_command = "rosrun mavros mavcmd long 183 11 1100 0 0 0 0 0"
    print("Disabling landing via command '%s'..." % disable_landing_command)
    os.system(disable_landing_command)

def set_mode(mode):
    print("Setting mode to %s..." % mode)
    response = set_mode_service(custom_mode=mode)
    while( not response.mode_sent ):
        print("\tSuccess: %s. Retrying..." % response.mode_sent)
        response = set_mode_service(custom_mode=mode)

def arm():
    print("Arming motors...")
    response = arm_service(value=True)
    while( not response.success ):
        time.sleep(1)
        response = arm_service(value=True)
        print("\tSuccess: %s. Retrying..." % response.success)

def takeoffcur():
    takeoff_command = "rosrun mavros mavcmd takeoffcur 0 0 10"
    print("Taking off via '%s'..." % takeoff_command)
    os.system(takeoff_command)

#######################################################################################

print("Starting test controller...")

signal.signal(signal.SIGINT, sigint_handler)

rospy.init_node('test_controller', anonymous=False)
mavros.set_namespace()

print("Waiting for ROS services...")
rospy.wait_for_service('/mavros/set_mode')
set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
rospy.wait_for_service('/mavros/cmd/arming')
arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
print("Found ROS services.")

print("Setting ROS subscribers.")
gps_fix_topic = mavros.get_topic('global_position', 'raw', 'fix')
rospy.Subscriber(gps_fix_topic, NavSatFix, position_callback)
rospy.Subscriber("/mavros/state", State, state_callback)
go_to_publisher = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)

disable_landing()

mode = "GUIDED"
print("Setting mode to %s..." % mode)
response = set_mode_service(custom_mode=mode)
while( not response.mode_sent ):
    print("\tSuccess: %s. Retrying..." % response.mode_sent)
    response = set_mode(custom_mode=mode)

arm()

takeoffcur()

time.sleep(15)

#set_mode("GUIDED_NOGPS")


enable_landing()

i = 0
while True and i < 15:

    distance = 0.5 * i

    os.system("rosrun mavros mavsetp local --position %s 200 10 1.6" % (distance))
    
    while armed:
        time.sleep(5)

    disable_landing()
    set_mode("GUIDED")
    arm()
    takeoffcur()
    
    # go home
    os.system("rosrun mavros mavsetp local --position %s 0 10 -1.6" % (distance))
    time.sleep(10)
    
    enable_landing()
    
    i += 1

#print("Spinning...")
#rospy.spin()
#command.
