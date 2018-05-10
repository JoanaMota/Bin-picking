
#!/usr/bin/env python
"""

This script will write a new file /blackbot_calibration/calibration/hand_in_eye.urdf.xacro with data collected from the ros system running immediately after the calibration.
It calls rosparams to find out the propper reference frames, and finds the transformation estimated through calibration using a tf listener. Then it will write the a xacro file similar to this
-----------------------------------
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
<!--This xacro contains six xacro propperties representing a transformation obtained through calibration-->
<!--Calibration generated at 01:28:16, 12/06/2016 -->
<!--This file was written automatically using the "rosrun blackbot_calibration store_calibration.py" command. -->
<xacro:property name="roll" value="0.0217361470355"/>
<xacro:property name="pitch" value="0.061872023059"/>
<xacro:property name="yaw" value="-0.0285727638957"/>
<xacro:property name="x" value="-0.0188701273242"/>
<xacro:property name="y" value="0.0954976920077"/>
<xacro:property name="z" value="0.0916589979185"/>
</robot>
-----------------------------------
"""

#########################
##    IMPORT MODULES   ##
#########################
import rospy
import roslib
import math
import tf
import rospkg
import time

#########################
##      HEADER         ##
#########################
__author__ = "Miguel Riem de Oliveira"
__date__ = "June 2016"
__copyright__ = "Copyright 2016, blackbot"
__credits__ = ["Miguel Riem de Oliveira"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Miguel Oliveira"
__email__ = "m.riem.oliveira@gmail.com"
__status__ = "Development"


#########################
## FUNCTION DEFINITION ##
#########################

##########
## MAIN ##
##########

if __name__ == "__main__":

    #--------------------#
    ### Initialization ###
    #--------------------#

    rospy.init_node('store_calibration')
    rospack = rospkg.RosPack()
    listener = tf.TransformListener()
    rospy.sleep(0.1)

    ### Get the reference frames for the estimated transformation ###
    print("Getting parameter /aruco_tracker/reference_frame")
    tracker_reference_frame = rospy.get_param("/aruco_tracker/reference_frame")

    print("Getting parameter /hand_eye_connector/camera_parent_frame")
    camera_parent_frame = rospy.get_param("/hand_eye_connector/camera_parent_frame")

    ### Get the transformation estimated from calibration ###
    t = rospy.Time()
    print("Waiting for transform from " + tracker_reference_frame + " to " + camera_parent_frame + "\n Will wait a maximum 10 seconds. Note that if you are using the calibration with interactive=true, you add a new calibration instance by pressing enter twice. Only then a transform with the calibration is published.")
    listener.waitForTransform(camera_parent_frame, tracker_reference_frame, t, rospy.Duration(5.0))
    try:
        (trans,rot) = listener.lookupTransform(camera_parent_frame, tracker_reference_frame, t)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Could not get transform, cannot save calibration.")
        exit()

    print("Collected the transform, will write the xacro.")
    ### store in local variables (and convert quaternion to rpy) ###
    eul = tf.transformations.euler_from_quaternion(rot, axes='sxyz')
    roll = eul[0]
    pitch = eul[1]
    yaw = eul[2]
    x = trans[0]
    y = trans[1]
    z = trans[2]


    ### write to a xml ###
    filename = rospack.get_path('blackbot_calibration') + "/calibration/hand_in_eye.urdf.xacro"
    fo = open(filename, "wb")

    fo.write('<?xml version="1.0"?>\n')
    fo.write('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n')
    fo.write('<!--This xacro contains six xacro propperties representing a transformation obtained through calibration-->\n')
    t = (time.strftime("%H:%M:%S, %d/%m/%Y"))
    fo.write('<!--Calibration generated at ' + t +' -->\n' )
    fo.write('<xacro:include filename="$(find bin_picking)/urdf/binpicking_robot_macro.xacro"/>\n')
    fo.write('<xacro:property name="roll" value="' + str(roll) + '"/>\n' )
    fo.write('<xacro:property name="pitch" value="' + str(pitch) + '"/>\n' )
    fo.write('<xacro:property name="yaw" value="' + str(yaw) + '"/>\n' )
    fo.write('<xacro:property name="x" value="' + str(x) + '"/>\n' )
    fo.write('<xacro:property name="y" value="' + str(y) + '"/>\n' )
    fo.write('<xacro:property name="z" value="' + str(z) + '"/>\n' )
    fo.write('</robot>\n' )

    <!-- include the kinetic xacro -->
    <xacro:include filename="$(find hector_xacro_tools)/urdf/inertia_tensors.urdf.xacro"/>
    <xacro:include filename="$(find hector_sensors_description)/urdf/kinect_camera.urdf.xacro" />

    <!-- macro defining the whole robot -->
    <xacro:macro name="bin_picking" params="prefix">
        <xacro:bin_picking_manipulator prefix="${prefix}" />
        <xacro:kinect_camera name="camera" parent="${prefix}robot_link_4">

             <!-- joana convert euler_from_quaternion 1 x para a frente melhor em y-->
            <!-- <origin
                xyz="0.273824339688 0.11318137131 -0.0197138711695"
                rpy="1.43637243915 1.55845880529 2.99165474735" /> -->
            <!-- <origin
                xyz="0.253824339688 0.11318137131 -0.0197138711695"
                rpy="1.43637243915 1.55845880529 2.99165474735" /> -->

             <!-- joana convert euler_from_quaternion 2-->
            <!-- <origin
                xyz="0.287897942007 0.113658162424 -0.0172542425314"
                rpy="1.30973391552 1.5275728369 2.88949861591" /> -->

             <!-- joana convert euler_from_quaternion 3-->
            <!-- <origin
                xyz="0.279609095669 0.116639748561 -0.0214942133212"
                rpy="1.92575936385 1.54891520166 -2.79157664793" /> -->
             
             <!-- joana convert euler_from_quaternion 4-->
            <!-- <origin
                xyz="0.268440466484 0.122899970085 -0.0166416582814"
                rpy="2.06885061027 1.55781252086 -2.64482979539" /> -->

             <!-- joana convert euler_from_quaternion 5 noite x para tras-->
            <!-- <origin
                xyz="0.247839774882 0.11700204948 -0.0190325324095"
                rpy="1.7398943203 1.54554241269 -2.98878736842" /> -->
             
             <!-- joana convert euler_from_quaternion 6 noite-->
            <!-- <origin
                xyz="0.230029470617 0.132857372442 -0.00630274821251"
                rpy="2.73368395102 1.54572900105 -1.94362403975" /> -->

             <!-- joana convert euler_from_quaternion 7 manha-->
            <!-- <origin
                xyz="0.276754546937 0.120140773748 -0.0143116801325"
                rpy="2.1200457846 1.54862201713 -2.59894915819" /> -->

             <!-- joana convert euler_from_quaternion 8 Miguel BOM x tarde-->
            <!-- <origin
                xyz="0.251399988337 0.121134021845 -0.0214740394396"
                rpy="2.77817146391 1.54118595996 -1.97217228011" /> -->
            
             <!-- joana convert euler_from_quaternion 9 MAL tarde-->
            <!-- <origin
                xyz="0.290023777948 0.117765053501 -0.0224434855418"
                rpy="1.76699764666 1.54179041055 -2.95878204774" /> -->
            
             <!-- joana convert euler_from_quaternion 10 tarde-->
            <!-- <origin
                xyz="0.243917673784 0.131550407348 -0.0194209143795"
                rpy="2.53224846805 1.54202866651 -2.18035254058" /> -->

             <!-- joana convert euler_from_quaternion 10 batota-->
            <!-- <origin
                xyz="0.251399988337 0.11318137131 -0.0194209143795"
                rpy="2.53224846805 1.54202866651 -2.18035254058" /> -->

             <!-- joana convert euler_from_quaternion 11 -->
            <!-- <origin
                xyz="0.279007993329 0.117180125822 -0.0180197530052"
                rpy="1.85013384122 1.54604862537 -2.87502917486" /> -->

             <!-- joana convert euler_from_quaternion 12 noite -->
            <!-- <origin
                xyz="0.2766913732440 0.117286498686 -0.0215517328186"
                rpy="1.79512358539 1.5416251848 -2.91911484011" /> -->

             <!-- joana convert euler_from_quaternion 12 noite batota-->
            <!-- <origin
                xyz="0.268440466484 0.11318137131 -0.0215517328186"
                rpy="1.79512358539 1.5416251848 -2.91911484011" /> -->

             <!-- joana convert euler_from_quaternion 13 noite -->
            <origin
                xyz="0.281486400201 0.116789579978 -0.0224725143874"
                rpy="1.76283050436 1.54019722186 -2.95054985582" />

                


                <!-- 
                X entre 0.268440466484 e 0.251399988337
                Y entre 0.117286498686 e 0.11318137131
                 -->

        </xacro:kinect_camera>
    </xacro:macro>
     
    # Close opend file
    fo.close()


    print('Estimated calibration is stored. You may run the calibrated system with\nroslaunch blackbot_bringup bringup.launch calibrated:=true\n')


