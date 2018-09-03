#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from math import pi
import numpy as np
# import tf
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3, Pose2D, Pose, PointStamped, Quaternion
from std_msgs.msg  import Float32, Header
import roslaunch
import math
from bin_picking.msg import TargetsPose
from robonuc.msg import ios

from generate_plan_move import generate_plan, move_robot

# initialize moveit_commander and rospy
print "============ Starting movement setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('bin_picking_move_fanuc',
                anonymous=True)

# This RobotCommander object is an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()

# This PlanningSceneInterface object is an interface to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

# MoveGroupCommander object. This object is an interface to one group of joints. 
# In this case the group is the joints in the manipulator. This interface can be used
# to plan and execute motions on the manipulator.
group = moveit_commander.MoveGroupCommander("manipulator")

rate = rospy.Rate(10) # 10hz

# Publisher of pointStamped of the grasping point
grasping_point_pub = rospy.Publisher(
                                    '/graspingPoint',
                                    PointStamped,
                                    queue_size = 10)
                                    # Publisher of pointStamped of the grasping point
io_pub = rospy.Publisher(
                        '/io_client_messages',
                        ios,
                        queue_size = 10)

normal = Vector3()
approx_point = Vector3()
eef_position_laser = Vector3()
roll = np.pi
pitch = Float32()
yaw = Float32()
laser_reading = Float32()

# Function for sending a ROS msg to the vs_IO_client.cpp node responsable for altering the state of the IOs
# function - 1 to read, 2 to switch on and 3 to switch of the respective IO number (ionumber)
def monitoring_ios(function,ionumber):
    cod = function*10 + ionumber
    io_msg = ios()
    io_msg.code = cod
    print "Setting IOs code:"
    print cod
    io_pub.publish(io_msg)


def callback_targets_pose(targets_pose):

    normal.x = targets_pose.normal.x
    normal.y = targets_pose.normal.y
    normal.z = targets_pose.normal.z

    approx_point.x = targets_pose.approx_point.x
    approx_point.y = targets_pose.approx_point.y
    approx_point.z = targets_pose.approx_point.z

    eef_position_laser.x = targets_pose.eef_position.x
    eef_position_laser.y = targets_pose.eef_position.y
    eef_position_laser.z = targets_pose.eef_position.z

    pitch.data = targets_pose.euler_angles.y
    yaw.data = targets_pose.euler_angles.x

def callback_laser_sensor(output_laser_reading):
    
    laser_reading.data = output_laser_reading.data

rospy.Subscriber("/targets_pose", TargetsPose, callback_targets_pose)
rospy.Subscriber("/output_laser_sensor", Float32, callback_laser_sensor)

group.set_planning_time(10.0)
group.set_planner_id("RRTConnectkConfigDefault")

visualization_point = Vector3()
visualization_point.x = 0.440
visualization_point.y = 0.000
visualization_point.z = 0.440 

# Quaternions of the Euler angles
quaternion_init = quaternion_from_euler(-np.pi, 0, roll)

# 1st POSITION - Visualize Workspace
# GENERATING PLAN
plan1, fraction1 = generate_plan(group, visualization_point, 5, quaternion_init)

# MOVEMENT
move_robot(plan1, fraction1, group)

# Launch objDetection and pointTFtransfer nodes
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_objDetect_pointTF = roslaunch.parent.ROSLaunchParent(uuid, ["/home/joana/catkin_ws/src/Bin-picking/bin_picking/launch/objDetection_pointTFtranfer.launch"])
# Start Launch node objDetection and pointTFtransfer
launch_objDetect_pointTF.start()

rospy.wait_for_message("/targets_pose", TargetsPose)

#Stop Launch node objDetection and pointTFtransfer
launch_objDetect_pointTF.shutdown()
# after having stopped both nodes the subscribed topics will be the last published and will be a fixed value

# Quaternions of the Euler angles
quaternion = quaternion_from_euler( roll, math.radians(-pitch.data), math.radians(-yaw.data), 'rzyx')

# 2nd POSITION - Measure with laser sensor
# GENERATING PLAN
plan2, fraction2 = generate_plan(group, eef_position_laser, 5, quaternion)

# MOVING
move_robot(plan2, fraction2, group)
if raw_input("If you want to EXIT press 'e': ") == 'e' :
    exit()

uuid3 = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid3)
launch_sensorRS232 = roslaunch.parent.ROSLaunchParent(uuid3, ["/home/joana/catkin_ws/src/Bin-picking/bin_picking/launch/sensorRS232.launch"])
# Start Launch node sensorRS232
launch_sensorRS232.start()

rospy.wait_for_message("/output_laser_sensor", Float32)
# Stop Launch node sensorRS232
launch_sensorRS232.shutdown()

print laser_reading.data
laser_reading_float = laser_reading.data + 1.000
# for vertical eggs
# laser_reading_float = laser_reading.data + 1.800

grasping_point = Vector3()
# + or -
grasping_point.x = approx_point.x + laser_reading_float * 0.001 * normal.x
grasping_point.y = approx_point.y + laser_reading_float * 0.001 * normal.y
grasping_point.z = approx_point.z + laser_reading_float * 0.001 * normal.z

# 3rd POSITION - Approximation point
# GENERATING PLAN
plan3, fraction3 = generate_plan(group, approx_point, 5, quaternion)

# MOVING
move_robot(plan3, fraction3, group)

# 4th POSITION - Grasping point
# GENERATING PLAN
plan4, fraction4 = generate_plan(group, grasping_point, 5, quaternion)

# MOVING
move_robot(plan4, fraction4, group)
    
# IO number 8 activates the suction
# First activate IO number for for IO number 8 to work
monitoring_ios(2,4)
monitoring_ios(2,8)

# 5th POSITION -Return to Approximation point 
# GENERATING PLAN 5
plan5, fraction5 = generate_plan(group, approx_point, 5, quaternion)

# MOVING
move_robot(plan5, fraction5, group)

final_point = Vector3()
final_point.x = 0.440
final_point.y = -0.270
final_point.z = 0.200 

# Quaternions of the Euler angles
quaternion_final = quaternion_from_euler(-np.pi, 0, roll)

# 1st POSITION - Visualize Workspace
# GENERATING PLAN
plan6, fraction6 = generate_plan(group, final_point, 5, quaternion_final)

# MOVEMENT
move_robot(plan6, fraction6, group)


monitoring_ios(3,8)
monitoring_ios(3,4)
exit()