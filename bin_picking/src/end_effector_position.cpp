 #include "../../bin_picking/include/bin_picking/header_pcl.h"

#include "bin_picking/TargetsPose.h"

using namespace std;

static geometry_msgs::Vector3 approx_point_robot_base; 
// static geometry_msgs::Pose2D euler_angles; 
float yaw, pitch;

void targets_callback (const bin_picking::TargetsPose::ConstPtr& targets_pose)
{
    approx_point_robot_base = targets_pose->approx_point;
    yaw = targets_pose->euler_angles.x;
    pitch = targets_pose->euler_angles.y;
}


int main (int argc, char* argv[])
{
    // Initialize ROS
    ros::init (argc, argv, "bin_picking_end_effector_position");
    ros::NodeHandle node;

    // Subscribe centroid published by the objDetection.cpp
    ros::Subscriber targets_pose_sub = node.subscribe<bin_picking::TargetsPose> ("/targets_pose", 1, targets_callback);
    ros::spinOnce();

    ros::Publisher eef_position_laser_reading_pointStamped = node.advertise<geometry_msgs::PointStamped> ("/eef_position_laser_reading_point", 1);
    ros::Publisher targets_pose_pub = node.advertise<bin_picking::TargetsPose>("/targets_pose", 1);

    tf2_ros::Buffer tfBuffer; 
    tf2_ros::TransformListener tfListener(tfBuffer);


    ros::Rate rate(10.0);
    while (node.ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped_laser;
        try
        {
 
            // APPROXIMATION POINT FOR LASER
            transformStamped_laser = tfBuffer.lookupTransform("ls_optical_frame", "eef_tool_tip", ros::Time(0),ros::Duration(3.0));
            // transformStamped_laser = tfBuffer.lookupTransform("eef_tool_tip", "ls_optical_frame", ros::Time(0),ros::Duration(3.0));
            cout << "Transformation from tool tip to laser emitter" << transformStamped_laser << endl;

            Eigen::Matrix4f T_eef_ls(4,4);
            Eigen::Quaterniond qua;
            qua.x() = transformStamped_laser.transform.rotation.x;
            qua.y() = transformStamped_laser.transform.rotation.y;
            qua.z() = transformStamped_laser.transform.rotation.z;
            qua.w() = transformStamped_laser.transform.rotation.w;
            Eigen::Matrix3d Rot = qua.normalized().toRotationMatrix();

            T_eef_ls(0,0) = Rot(0,0);
            T_eef_ls(0,1) = Rot(0,1);
            T_eef_ls(0,2) = Rot(0,2);
            T_eef_ls(1,0) = Rot(1,0);
            T_eef_ls(1,1) = Rot(1,1);
            T_eef_ls(1,2) = Rot(1,2);
            T_eef_ls(2,0) = Rot(2,0);
            T_eef_ls(2,1) = Rot(2,1);
            T_eef_ls(2,2) = Rot(2,2);
            T_eef_ls(0,3) = transformStamped_laser.transform.translation.x * 0;
            T_eef_ls(1,3) = transformStamped_laser.transform.translation.y * 0;
            T_eef_ls(2,3) = transformStamped_laser.transform.translation.z * 2;
            T_eef_ls(3,0) = 0;
            T_eef_ls(3,1) = 0;
            T_eef_ls(3,2) = 0;
            T_eef_ls(3,3) = 1;

            // cout << "Transformation from enf-effector to laser: \n" << T_eef_ls << endl; 

            // yaw = - yaw;
            // pitch = - pitch;
            
            Eigen::Matrix4f T_robot_point(4,4);
            // T_robot_point(0,0) = -cos(pitch);
            // T_robot_point(0,1) = -sin(yaw)*sin(pitch);
            // T_robot_point(0,2) = -cos(yaw)*sin(pitch);
            // T_robot_point(1,0) = 0;
            // T_robot_point(1,1) = -cos(yaw);
            // T_robot_point(1,2) = sin(yaw);
            // T_robot_point(2,0) = -sin(pitch);
            // T_robot_point(2,1) = cos(pitch)*sin(yaw);
            // T_robot_point(2,2) = cos(yaw)*cos(pitch);
            T_robot_point(0,0) = cos(pitch);
            T_robot_point(0,1) = sin(yaw)*sin(pitch);
            T_robot_point(0,2) = cos(yaw)*sin(pitch);
            T_robot_point(1,0) = 0;
            T_robot_point(1,1) = cos(yaw);
            T_robot_point(1,2) = -sin(yaw);
            T_robot_point(2,0) = -sin(pitch);
            T_robot_point(2,1) = cos(pitch)*sin(yaw);
            T_robot_point(2,2) = cos(yaw)*cos(pitch);
            T_robot_point(0,3) = approx_point_robot_base.x;
            T_robot_point(1,3) = approx_point_robot_base.y;
            T_robot_point(2,3) = approx_point_robot_base.z;
            T_robot_point(3,0) = 0;
            T_robot_point(3,1) = 0;
            T_robot_point(3,2) = 0;
            T_robot_point(3,3) = 1;

            // cout << "Transformation from robot base to approximation point: \n" << T_robot_point << endl; 

            Eigen::Matrix4f T_robot_eef(4,4);
            T_robot_eef = T_eef_ls.inverse() * T_robot_point;
            // T_robot_eef = T_robot_point * T_eef_ls;

            cout << "Transformation from robot base to End-effector: \n" << T_robot_eef << endl;

            geometry_msgs::Vector3 eef_position_laser_reading;
            eef_position_laser_reading.x = T_robot_eef(0,3);
            eef_position_laser_reading.y = T_robot_eef(1,3);
            eef_position_laser_reading.z = T_robot_eef(2,3);

            // cout << "End-effector position for laser reading: \n " << eef_position_laser_reading << endl;

            geometry_msgs::PointStamped eef_position_laser_reading_pt;
            eef_position_laser_reading_pt.point.x = eef_position_laser_reading.x;
            eef_position_laser_reading_pt.point.y = eef_position_laser_reading.y;
            eef_position_laser_reading_pt.point.z = eef_position_laser_reading.z;
            eef_position_laser_reading_pt.header.frame_id = "/robot_base_link";

            bin_picking::TargetsPose targets_pose;
            targets_pose.eef_position = eef_position_laser_reading;

            targets_pose_pub.publish(targets_pose);
            
            eef_position_laser_reading_pointStamped.publish(eef_position_laser_reading_pt);
            
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //will call all the callbacks waiting to be called at that point in time
        rate.sleep();
    }
}