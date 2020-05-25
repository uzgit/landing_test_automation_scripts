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
from geometry_msgs.msg import PoseStamped
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

landing_pad_relative_x = 0
landing_pad_relative_y = 0
landing_pad_relative_z = 0

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

def relative_pose_callback(data):

    global landing_pad_relative_x
    global landing_pad_relative_y
    global landing_pad_relative_z

    landing_pad_relative_x = data.pose.position.x
    landing_pad_relative_y = data.pose.position.y
    landing_pad_relative_z = data.pose.position.z

#    print("Estimated position: (%0.3f, %0.3f, %0.3f)" % ( landing_pad_relative_x, landing_pad_relative_y, landing_pad_relative_z ) )

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
pid_parameters_u_publisher = rospy.Publisher("/landing_controller/pid_parameters_u", Vector3, queue_size=10)
pid_parameters_yaw_publisher = rospy.Publisher("/landing_controller/pid_parameters_yaw", Vector3, queue_size=10)

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

column_names = ['time', 'iris_position_x', 'iris_position_y', 'iris_position_z', 'iris_linear_velocity_x', 'iris_linear_velocity_y', 'iris_linear_velocity_z', 'landing_pad_position_x', 'landing_pad_position_y', 'landing_pad_position_z', 'landing_pad_linear_velocity_x', 'landing_pad_linear_velocity_y', 'landing_pad_linear_velocity_z', 'landing_phase', 'plane_displacement', 'energy_consumed', 'latitude', 'longitude', 'altitude', 'landing_pad_relative_x', 'landing_pad_relative_y', 'landing_pad_relative_z']

# subscribers for logging
rospy.Subscriber("/gazebo/model_states/", ModelStates, model_states_callback)
rospy.Subscriber("/mavros/battery", BatteryState, battery_callback)
rospy.Subscriber("/landing_pad/plane_displacement", Float64, plane_displacement_callback)
rospy.Subscriber("/landing_phase", UInt8, landing_phase_callback)
rospy.Subscriber("/landing_pad/relative_pose_stamped", PoseStamped, relative_pose_callback)

# disable_landing()
set_mode("GUIDED")
arm()
#takeoffcur()
#time.sleep(10)


# we assume that before the first landing run starts, the drone is hovering directly above the landing pad
def landing_run(kpu, kiu, kdu):

    print("Reconfiguring PID parameters...")
    pid_parameters_u_publisher.publish(Vector3(kpu, 0, kdu))

    one_second = rospy.Rate(1)
    loop_rate = rospy.Rate(20)
    success = False
    for i in range(5):
        
        # create a place to store the data
        dataframe = pandas.DataFrame(columns = column_names)

        # so that the drone will land
        enable_landing()
        start_time = rospy.Time.now()
        
        while armed and (rospy.Time.now() - start_time < rospy.Duration(45)):
            
            # save a single row of data
            values = [rospy.get_time(), iris_position_x, iris_position_y, iris_position_z, iris_linear_velocity_x, iris_linear_velocity_y, iris_linear_velocity_z, landing_pad_position_x, landing_pad_position_y, landing_pad_position_z, landing_pad_linear_velocity_x, landing_pad_linear_velocity_y, landing_pad_linear_velocity_z, landing_phase, plane_displacement, energy_consumed, latitude, longitude, altitude, landing_pad_relative_x, landing_pad_relative_y, landing_pad_relative_z]
            new_row = dict(zip(column_names, values))
            dataframe = dataframe.append(new_row, ignore_index=True)
            loop_rate.sleep()

        if (rospy.Time.now() - start_time < rospy.Duration(60)):
            success = True

        dataframe.to_csv("/home/joshua/Documents/pid_u_testing/test_%0.2f,%0.2f,%0.2f_%s_%s.csv" % (kpu, kiu, kdu, i, success))

        # so that we can take off
        disable_landing()
        # takeoff at the current location
        while( iris_position_z < 8 ):
            set_mode("GUIDED")
            arm()
            takeoffcur()
            # approach the landing pad, facing forwards
            os.system("rosrun mavros mavsetp local --position 0 30 10 1.6")
            one_second.sleep()

        for num_seconds in range(3):
            one_second.sleep()

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

def run(kp3, ki3, kd3, kp2, ki2, kd2, kp1, ki1, kd1):

    global plane_displacement

    # reconfigure PID parameters
    print("Reconfiguring PID parameters...")
    pid_parameters_1_publisher.publish(Vector3(kp1, ki1, kd1))
    pid_parameters_2_publisher.publish(Vector3(kp2, ki2, kd2))
    pid_parameters_3_publisher.publish(Vector3(kp3, ki3, kd3))

    one_second = rospy.Rate(1)
    loop_rate = rospy.Rate(20)
    for i in range(5):

        # so that we can control the position
        disable_landing()

        # go to starting point, facing backwards

        os.system("rosrun mavros mavsetp local --position %s 0 10 -1.6" % (i))
        while( numpy.sqrt((iris_position_x) ** 2 + (iris_position_y + i) ** 2) > 2 ):
            loop_rate.sleep()
        one_second.sleep()

        # create a place to store the data
        dataframe = pandas.DataFrame(columns = column_names)

        # enable the landing controller (ensure it does not use descent or yaw correction in this case)
        enable_landing()

        # approach the landing pad, facing forwards
        os.system("rosrun mavros mavsetp local --position %s 200 10 1.6" % (i))

        start_time = rospy.Time.now()
        success_start_time = rospy.Time.now()
        success = False

        while (rospy.Time.now() - start_time < rospy.Duration(45)) and (not success):

            # save a single row of data
            values = [rospy.get_time(), iris_position_x, iris_position_y, iris_position_z, iris_linear_velocity_x, iris_linear_velocity_y, iris_linear_velocity_z, landing_pad_position_x, landing_pad_position_y, landing_pad_position_z, landing_pad_linear_velocity_x, landing_pad_linear_velocity_y, landing_pad_linear_velocity_z, landing_phase, plane_displacement, energy_consumed, latitude, longitude, altitude]
            new_row = dict(zip(column_names, values))
            dataframe = dataframe.append(new_row, ignore_index=True)

            if( plane_displacement > 1.6 ):
                success_start_time = rospy.Time.now()

            if( rospy.Time.now() - success_start_time >= rospy.Duration(3) ):
                success = True

            loop_rate.sleep()

        plane_displacement = 999

        dataframe.to_csv("/home/joshua/Documents/pid_testing/test_%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f_%s_%s.csv" % (kp3, ki3, kd3, kp2, ki2, kd2, kp1, ki1, kd1, i, success))

        if( success ):
            print("Success")
        else:
            print("Timeout. Moving on...")

def radial_landing_run(theta, radius, up_position):

    print(theta)

    dataframe = pandas.DataFrame(columns = column_names)

    east_position  = radius * numpy.cos(theta)
    north_position = radius * numpy.sin(theta) + 30
    goto_wait(east_position, north_position, up_position, theta)

    enable_landing()

    east_position  = radius * numpy.cos(theta + numpy.pi)
    north_position = radius * numpy.sin(theta + numpy.pi) + 30
    goto(east_position, north_position, up_position, theta + numpy.pi)

    while armed:
        # save a single row of data
        values = [rospy.get_time(), iris_position_x, iris_position_y, iris_position_z, iris_linear_velocity_x, iris_linear_velocity_y, iris_linear_velocity_z, landing_pad_position_x, landing_pad_position_y, landing_pad_position_z, landing_pad_linear_velocity_x, landing_pad_linear_velocity_y, landing_pad_linear_velocity_z, landing_phase, plane_displacement, energy_consumed, latitude, longitude, altitude, landing_pad_relative_x, landing_pad_relative_y, landing_pad_relative_z]
        new_row = dict(zip(column_names, values))
        dataframe = dataframe.append(new_row, ignore_index=True)
        
        time.sleep(0.1)

    dataframe.to_csv("/home/joshua/Documents/radial_land_testing/theta_%0.3f.csv" % theta)

    arm()
    disable_landing()
    takeoffcur()


radius = 30
up_position = 10

n = 10
#thetas_raw = range(0, 315, int(315 / n))
thetas_raw = range(0, 631, int(631 / n))
thetas = []
for i in range(len(thetas_raw)):
    thetas.append( thetas_raw[i] / (10 * n))
#    thetas.append(2 * thetas_raw[i] / (10 * n))
#    thetas.append( numpy.pi + 2 * (thetas_raw[i] / (10 * n)))

print(thetas)

#for i in range(0, 631, 63):
disable_landing()
for theta in thetas:

    radial_landing_run(theta, radius, up_position)
'''
    print(theta)
    
    east_position  = radius * numpy.cos(theta)
    north_position = radius * numpy.sin(theta) + 30
    goto_wait(east_position, north_position, up_position, theta)

    enable_landing()

    east_position  = radius * numpy.cos(theta + numpy.pi)
    north_position = radius * numpy.sin(theta + numpy.pi) + 30
    goto(east_position, north_position, up_position, theta + numpy.pi)

    while armed:
        time.sleep(1)

    arm()
    disable_landing()
    takeoffcur()
'''

#    command = "rosrun mavros mavsetp local --position %0.3f %0.3f 10 %0.3f" % ( east_position, north_position, theta)
#    print(command)
#    os.system( command )
   
#    east_position  = -radius * numpy.cos(theta)
#    north_position = -radius * numpy.sin(theta) + 30
#    goto(-east_position, -north_position, up_position, theta + numpy.pi)
