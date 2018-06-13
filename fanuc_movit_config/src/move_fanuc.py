#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np
# import tf
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler

# initialize moveit_commander and rospy
print "============ Starting movement setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
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
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting movement "

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Name of the end-effector link: %s" % group.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"


# Plan a motion for this group to a desired pose for the end-effector
print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()

pose_target.orientation.x = 0.0 
pose_target.orientation.y = 1
pose_target.orientation.z = 0.0
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = -0.05
pose_target.position.z = 1.1
group.set_pose_target(pose_target)

plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)

print "============ Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory)

print "============ Waiting while plan1 is visualized (again)..."
rospy.sleep(5)

# Points for a clean path
waypoints = []

# start with the current pose
waypoints.append(group.get_current_pose().pose)

# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.10
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))

(plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)

if fraction == 1.0:
    print 'planning was successful'
    raw_input()
    group.execute(plan3)
else:
    print 'planning was not successfull'
print fraction


# q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx')

# q = quaternion_from_euler(roll, pitch, yaw)

# print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])

# cr = cos(roll * 0.5)
# sr = sin(roll * 0.5)
# cp = cos(pitch * 0.5)
# sp = sin(pitch * 0.5)
# cy = cos(yaw * 0.5)
# sy = sin(yaw * 0.5)

# q.w() = cr * cy * cp + sr * sy * sp
# q.z() = sr * cy * cp - cr * sy * sp
# q.x() = cr * sy * cp - sr * cy * sp
# q.y() = cr * cy * sp + sr * sy * cp
