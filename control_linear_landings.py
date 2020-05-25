#!/usr/bin/env python3

import signal
import sys
import os
import time
import math

import numpy
import pandas

import rospy
import mavros
from mavros import command

from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import State
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64
from std_msgs.msg import UInt8

iris_position_x = 0
iris_position_y = 0
iris_position_z = 0
iris_linear_velocity_x = 0
iris_linear_velocity_y = 0
iris_linear_velocity_z = 0
landing_pad_position_x = 0
landing_pad_position_y = 0
landing_pad_position_z = 0
landing_pad_linear_velocity_x = 0
landing_pad_linear_velocity_y = 0
landing_pad_linear_velocity_z = 0

landing_phase = -1
energy_consumed = -1
plane_displacement = 999

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

def model_states_callback(data):

    global iris_position_x
    global iris_position_y
    global iris_position_z
    global iris_linear_velocity_x
    global iris_linear_velocity_y
    global iris_linear_velocity_z
    global landing_pad_position_x
    global landing_pad_position_y
    global landing_pad_position_z
    global landing_pad_linear_velocity_x
    global landing_pad_linear_velocity_y
    global landing_pad_linear_velocity_z

    iris_index = data.name.index("iris_demo")
    iris_position_x = data.pose[iris_index].position.x
    iris_position_y = data.pose[iris_index].position.y
    iris_position_z = data.pose[iris_index].position.z
    iris_linear_velocity_x = data.twist[iris_index].linear.x
    iris_linear_velocity_y = data.twist[iris_index].linear.y
    iris_linear_velocity_z = data.twist[iris_index].linear.z
    
    landing_pad_index = data.name.index("landing_pad_combo")
    landing_pad_position_x = data.pose[landing_pad_index].position.x
    landing_pad_position_y = data.pose[landing_pad_index].position.y
    landing_pad_position_z = data.pose[landing_pad_index].position.z
    landing_pad_linear_velocity_x = data.twist[landing_pad_index].linear.x
    landing_pad_linear_velocity_y = data.twist[landing_pad_index].linear.y
    landing_pad_linear_velocity_z = data.twist[landing_pad_index].linear.z

def landing_phase_callback(data):
    global landing_phase
    landing_phase = data.data

def battery_callback(data):

    global energy_consumed
    energy_consumed = data.capacity # edited just for this purpose

def plane_displacement_callback(data):

    global plane_displacement
    plane_displacement = data.data

pid_parameters_1_publisher = rospy.Publisher("/landing_controller/pid_parameters_1", Vector3, queue_size=10)
pid_parameters_2_publisher = rospy.Publisher("/landing_controller/pid_parameters_2", Vector3, queue_size=10)
pid_parameters_3_publisher = rospy.Publisher("/landing_controller/pid_parameters_3", Vector3, queue_size=10)


    

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

column_names = ['time', 'iris_position_x', 'iris_position_y', 'iris_position_z', 'iris_linear_velocity_x', 'iris_linear_velocity_y', 'iris_linear_velocity_z', 'landing_pad_position_x', 'landing_pad_position_y', 'landing_pad_position_z', 'landing_pad_linear_velocity_x', 'landing_pad_linear_velocity_y', 'landing_pad_linear_velocity_z', 'landing_phase', 'plane_displacement', 'energy_consumed', 'latitude', 'longitude', 'altitude']

# subscribers for logging
rospy.Subscriber("/gazebo/model_states/", ModelStates, model_states_callback)
rospy.Subscriber("/mavros/battery", BatteryState, battery_callback)
rospy.Subscriber("/landing_pad/plane_displacement", Float64, plane_displacement_callback)
rospy.Subscriber("/landing_phase", UInt8, landing_phase_callback)

# disable_landing()
set_mode("GUIDED")
arm()
#takeoffcur()
#time.sleep(10)

def goto(east_position, north_position, up_position, theta):

    command = "rosrun mavros mavsetp local --position %0.3f %0.3f %0.3f %0.3f" % ( east_position, north_position, up_position, theta)
    print(command)
    os.system( command )

def goto_wait(east_position, north_position, up_position, theta):

    goto(east_position, north_position, up_position, theta)

    waypoint_radius = 1
    distance = waypoint_radius + 1
    while( distance > waypoint_radius ):
        distance = math.sqrt( (-iris_position_y - east_position) ** 2 + (iris_position_x - north_position) ** 2 + (iris_position_z - up_position) ** 2 )
#        print("Iris position: (%0.3f, %0.3f, %0.3f)" % ( -iris_position_y, iris_position_x, iris_position_z ) )
#        print(-iris_position_y - east_position)
#        print(iris_position_x - north_position)
#        print(iris_position_z - up_position)
        print("Distance to waypoint: %0.3f" % (distance) )
        time.sleep(0.5)
    time.sleep(5)

#def run(kp3, ki3, kd3, kp2, ki2, kd2, kp1, ki1, kd1):
def run():
    global plane_displacement

    # go to starting point, facing backwards
    goto_wait(0, 0, 10, -1.6)

    # create a place to store the data
    dataframe = pandas.DataFrame(columns = column_names)

    # enable the landing controller (ensure it does not use descent or yaw correction in this case)
    enable_landing()

    # approach the landing pad, facing forwards
    goto(0, 200, 10, 1.6)

    loop_rate = rospy.Rate(20)
    while armed:

        # save a single row of data
        values = [rospy.get_time(), iris_position_x, iris_position_y, iris_position_z, iris_linear_velocity_x, iris_linear_velocity_y, iris_linear_velocity_z, landing_pad_position_x, landing_pad_position_y, landing_pad_position_z, landing_pad_linear_velocity_x, landing_pad_linear_velocity_y, landing_pad_linear_velocity_z, landing_phase, plane_displacement, energy_consumed, latitude, longitude, altitude]
        new_row = dict(zip(column_names, values))
        dataframe = dataframe.append(new_row, ignore_index=True)

        loop_rate.sleep()

    dataframe.to_csv("/home/joshua/Documents/linear_land_testing/test_%s.csv" % (i))

    arm()
    disable_landing()
    takeoffcur()

disable_landing()
for i in range(5):
    run()
