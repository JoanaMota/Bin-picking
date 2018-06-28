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

# DisplayTrajectory publisher which is used to publish trajectories for RVIZ to visualize.
# display_trajectory_publisher = rospy.Publisher(
#                                     '/move_group/display_planned_path',
#                                     moveit_msgs.msg.DisplayTrajectory,
#                                     queue_size=20)
# rospy.sleep(1)

header = Header()
header.stamp = rospy.Time.now() 
rate = rospy.Rate(10) # 10hz

# Publisher of pointStamped of the grasping point
grasping_point_pub = rospy.Publisher(
                                    '/graspingPoint',
                                    PointStamped,
                                    queue_size = 10)

print "============ Waiting for RVIZ..."
# rospy.sleep(10)
print "============ Starting movement "

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Name of the end-effector link: %s" % group.get_end_effector_link()

print "============ Robot Groups: %s" %robot.get_group_names()

# print "============ Printing robot state"
# print robot.get_current_state()
# print "============"

normal = Vector3()
approx_point = Vector3()
eef_position_laser = Vector3()
# roll = 0.0
roll = np.pi
pitch = Float32()
yaw = Float32()
laser_reading = Float32()

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
    laser_reading.data = output_laser_reading

rospy.Subscriber("/targets_pose", TargetsPose, callback_targets_pose)
rospy.Subscriber("/output_laser_sensor", Float32, callback_laser_sensor)

print "============ Generating plan 1 = 1st POSITION - Visualize Workspace  ============"
group.set_planning_time(10.0)
group.set_planner_id("RRTConnectkConfigDefault")

visualization_point = Vector3()
visualization_point.x = 0.440
visualization_point.y = 0.0
visualization_point.z = 0.440 

# Quaternions of the Euler angles
quaternion_init = quaternion_from_euler(-np.pi, 0, roll)
# quaternion_init = quaternion_from_euler(-160*np.pi/180, -7.53*np.pi/180, roll)
print "The quaternion representation is %s %s %s %s." % (quaternion_init[0], quaternion_init[1], quaternion_init[2], quaternion_init[3])

# exit()
n_points1 = 5
# GENERATING PLAN
plan1, fraction1 = generate_plan(group, visualization_point, n_points1, quaternion_init)

# MOVEMENT
move_robot(plan1, fraction1, group)

print "============ MOVING plan 1 = 1st POSITION - Visualize Workspace  ============"
print "When the robot STOPS moving press any key to continue!"
if raw_input("If you want to EXIT press e: ") == 'e' :
    exit()


# Launch objDetection and pointTFtransfer nodes
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_objDetect_pointTF = roslaunch.parent.ROSLaunchParent(uuid, ["/home/joana/catkin_ws/src/Bin-picking/bin_picking/launch/objDetection_pointTFtranfer.launch"])
# Start Launch node objDetection and pointTFtransfer
launch_objDetect_pointTF.start()

# star to subscribe to all topics even though sensorRS232 have not been yet launched
# rospy.spin()

print "=== Running node objDetection and pointTFtransfer "

# WAIT for "a" to be pressed
print "Normal: "
print normal
print "Approximation Point: "
print approx_point
print "End-effector Position for laser sensor measurement: "
print eef_position_laser
print "Euler Angles: "
print "yaw: ", yaw.data, " pitch: ", pitch.data
while raw_input('') != 'a':
    print "Normal: "
    print normal
    print "Approximation Point: "
    print approx_point
    print "End-effector Position for laser sensor measurement: "
    print eef_position_laser
    print "Euler Angles: "
    print "yaw: ", yaw.data, " pitch: ", pitch.data
    print "Press A to Confirm Values and Continue!!!!!"

#Stop Launch node objDetection and pointTFtransfer
launch_objDetect_pointTF.shutdown()
# after having stopped both nodes the subscribed topics will be the last published and will be a fixed value

if raw_input("Are the points OK ??? If NOT press n to relaunch !!!") == 'n' :
    # Launch objDetection and pointTFtransfer nodes
    uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid2)
    launch_objDetect_pointTF2 = roslaunch.parent.ROSLaunchParent(uuid2, ["/home/joana/catkin_ws/src/Bin-picking/bin_picking/launch/objDetection_pointTFtranfer.launch"])
    launch_objDetect_pointTF2.start()
    print "=== Running node objDetection and pointTFtransfer "

    # WAIT for "a" to be pressed
    print "Normal: "
    print normal
    print "Approximation Point: "
    print approx_point
    print "End-effector Position for laser sensor measurement: "
    print eef_position_laser
    print "Euler Angles: "
    print "yaw: ", yaw.data, " pitch: ", pitch.data
    while raw_input('') != 'a':
        print "Normal: "
        print normal
        print "Approximation Point: "
        print approx_point
        print "End-effector Position for laser sensor measurement: "
        print eef_position_laser
        print "Euler Angles: "
        print "yaw: ", yaw.data, " pitch: ", pitch.data
        print "Press A to Confirm Values and Continue!!!!!"

    #Stop Launch node objDetection and pointTFtransfer
    launch_objDetect_pointTF2.shutdown()
    
print "============ Generating plan 2 = 2nd POSITION - Measure with laser sensor ============"   

# Quaternions of the Euler angles
quaternion = quaternion_from_euler( roll, math.radians(-pitch.data), math.radians(-yaw.data), 'rzyx')
print "The quaternion representation is %s %s %s %s." % (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
# quaternion[0] = -quaternion[0]
# quaternion[1] = -quaternion[1]
# quaternion[2] = -quaternion[2]
# quaternion[3] = -quaternion[3]

q = Quaternion()

cr = math.cos(math.radians(yaw.data) * 0.5)
sr = math.sin(math.radians(yaw.data) * 0.5)
cp = math.cos(math.radians(pitch.data) * 0.5)
sp = math.sin(math.radians(pitch.data) * 0.5)
cy = math.cos(roll * 0.5)
sy = math.sin(roll * 0.5)

q.w = cr * cy * cp + sr * sy * sp
q.z = sr * cy * cp - cr * sy * sp
q.x = cr * sy * cp - sr * cy * sp
q.y = cr * cy * sp + sr * sy * cp

quaternion2 = [ q.x, q.y, q.z, q.w ]

print "The quaternion representation is %s %s %s %s." % (quaternion2[0], quaternion2[1], quaternion2[2], quaternion2[3])

# # GENERATING PLAN
# plan3, fraction3 = generate_plan(group, approx_point, 5, quaternion)

# # MOVING
# move_robot(plan3, fraction3, group)
# print "============ MOVING plan 3 = 3rd POSITION - Approximation point ============"
# print "When the robot STOPS moving press any key to continue!"
# if raw_input("If you want to EXIT press e: ") == 'e' :
#     exit()


# GENERATING PLAN
plan2, fraction2 = generate_plan(group, eef_position_laser, 5, quaternion)

# MOVING
move_robot(plan2, fraction2, group)

print "============ MOVING plan 2 = 2nd POSITION - Measure with laser sensor ============"
print "When the robot STOPS moving press ENTER to continue!"
if raw_input("If you want to EXIT press e: ") == 'e' :
    exit()

uuid3 = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid3)
launch_sensorRS232 = roslaunch.parent.ROSLaunchParent(uuid3, ["/home/joana/catkin_ws/src/Bin-picking/bin_picking/launch/sensorRS232.launch"])
# Start Launch node sensorRS232
launch_sensorRS232.start()

print "=== Running node sensorRS232 "
print "If no value appear is because of Error. Could not open port!!"

print "Laser Reading: "
print laser_reading.data
while raw_input('') != 'b':
    print "Laser Reading: "
    print laser_reading.data
    print "Press B to Canfirm value and continue!!!!!"
    

# Stop Launch node sensorRS232
launch_sensorRS232.shutdown()

print "============ Generating plan 3 = 3rd POSITION - Approximation point  ============"    

print "=== Calculating Grasping point... "

laser_reading_float = laser_reading.data

grasping_point = Vector3()
# + or -
grasping_point.x = approx_point.x + laser_reading_float.data * 0.001 * normal.x
grasping_point.y = approx_point.y + laser_reading_float.data * 0.001 * normal.y
grasping_point.z = approx_point.z + laser_reading_float.data * 0.001 * normal.z

print " ==== Grasping Point ===="
print grasping_point

# Creating and publishing a PointStamped of the grasping point for visualization
grasping_point_ps = PointStamped()
grasping_point_ps.header.frame_id = "/robot_base_link"
grasping_point_ps.point.x = grasping_point.x
grasping_point_ps.point.y = grasping_point.y
grasping_point_ps.point.z = grasping_point.z

grasping_point_pub.publish(grasping_point_ps)

print "Confirm Grasping Point!!!"
raw_input()

# GENERATING PLAN
plan3, fraction3 = generate_plan(group, approx_point, 5, quaternion)

# MOVING
move_robot(plan3, fraction3, group)
print "============ MOVING plan 3 = 3rd POSITION - Approximation point ============"
print "When the robot STOPS moving press any key to continue!"
if raw_input("If you want to EXIT press e: ") == 'e' :
    exit()

print "============ Generating plan 4 = 4th POSITION - Grasping point  ============"    
# GENERATING PLAN
plan4, fraction4 = generate_plan(group, grasping_point, 5, quaternion)

# MOVING
move_robot(plan4, fraction4, group)
print "============ MOVING plan 4 = 4th POSITION - Grasping point ============"
print "When the robot STOPS moving press any key to continue!"
if raw_input("If you want to EXIT press e: ") == 'e' :
    exit()
exit()

# ====================================================================================================================

print "============ SUCTION ============"
# SUCTION

print "============ Generating plan 5 = 5th POSITION -Return to Approximation point  ============"    
# GENERATING PLAN 5
plan5, fraction5 = generate_plan(group, approx_point, 5, quaternion)

# MOVING
move_robot(plan5, fraction5, group)

print "============ MOVING plan 5 = 5th POSITION -Return to Approximation point  ============"    
print "When the robot STOPS moving press any key to continue!"
if raw_input("If you want to EXIT press e: ") == 'e' :
    exit()
