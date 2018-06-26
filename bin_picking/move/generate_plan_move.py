#!/usr/bin/env python2
import copy
import rospy
import math
from geometry_msgs.msg import Vector3, Pose2D, Pose
import moveit_commander
import moveit_msgs.msg


def generate_plan(group, final_point, number_of_points, q):
    
    # Points for a clean path
    waypoints = []

    # Start with the current pose
    starting_point = Pose()
    starting_point = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(starting_point))

    vector = Vector3()
    vector.x = final_point.x - starting_point.position.x
    vector.y = final_point.y - starting_point.position.y
    vector.z = final_point.z - starting_point.position.z

    length = math.sqrt ( vector.x ** 2.0 + vector.y ** 2.0 + vector.z ** 2.0 )

    point = Pose()
    point.orientation.x = q[0]
    point.orientation.y = q[1]
    point.orientation.z = q[2]
    point.orientation.w = q[3]

    # calculating all the desired points for a safe plan movement
    for n in range(1, number_of_points+1):

        point.position.x = starting_point.position.x + n*(length/3) * vector.x
        point.position.y = starting_point.position.y + n*(length/3) * vector.y
        point.position.z = starting_point.position.z + n*(length/3) * vector.z

        # save all the points
        waypoints.append(copy.deepcopy(point))


    # Final Position
    final_position = Pose()
    final_position.orientation.x = q[0]
    final_position.orientation.y = q[1]
    final_position.orientation.z = q[2]
    final_position.orientation.w = q[3]
    final_position.position.x = final_point.x
    final_position.position.y = final_point.y
    final_position.position.z = final_point.z

    waypoints.append(copy.deepcopy(final_position))

    # We want the cartesian path to be interpolated at 
    # a resolution of 1 cm which is why we will specify 
    # 0.01 as the eef_step in cartesian translation. 
    # We will specify the jump threshold as 0.0, effectively
    # disabling it.
    plan, fraction = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step 
                                0.0)         # jump_threshold 
    for i in range(len(plan.joint_trajectory.points)-1):
        plan.joint_trajectory.points[i].time_from_start = rospy.Time.now() + plan.joint_trajectory.points[i].time_from_start
    
    print "=== Waiting while RVIZ displays plan..."
    rospy.sleep(5)

    return plan, fraction

def move_robot(plan, fraction, group):

    print "fraction: ", fraction
    if fraction == 1.0:
        print "planning was successful"
        print "============ IS PLAN OK??  "
        if raw_input("Should I MOVE???? If YES press y!!!!") == 'y' :
            # MOVEMENT
            group.execute(plan)
    else:
        print "planning was not successfull"


