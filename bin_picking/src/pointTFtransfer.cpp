#include "../../bin_picking/include/bin_picking/header_pcl.h"

#include "bin_picking/TargetsPose.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
float X, Y, Z, normal_X, normal_Y, normal_Z;

void cloud_centroid (const PointCloud::ConstPtr& cloud_centroid)
{
    X = cloud_centroid->points[0].x;
    Y = cloud_centroid->points[0].y;
    Z = cloud_centroid->points[0].z;
}

void cloud_centroid_normal (const PointCloudNormal::ConstPtr& cloud_normal)
{
    normal_X = cloud_normal->points[0].normal_x;
    normal_Y = cloud_normal->points[0].normal_y;
    normal_Z = cloud_normal->points[0].normal_z;
}

int main (int argc, char* argv[])
{
    // Initialize ROS
    ros::init (argc, argv, "bin_picking_pointTFtransfer");
    ros::NodeHandle node;

    // Subscribe centroid published by the objDetection.cpp
    ros::Subscriber sub_centroid = node.subscribe<PointCloud> ("/cloud_centroid", 1, cloud_centroid);
    // Subscribe normal published by the objDetection.cpp
    ros::Subscriber sub_normal = node.subscribe<PointCloudNormal> ("/cloud_centroid_normal", 1, cloud_centroid_normal);
    ros::spinOnce();

    //-------PUBLISH:
    // MSG with normal, approximation point, end-effector position for laser reading and euler angles
    ros::Publisher targets_pose_pub = node.advertise<bin_picking::TargetsPose>("/targets_pose", 1);
    // For visualization
    // Centroid for visualization
    ros::Publisher centroid_pointStamped = node.advertise<geometry_msgs::PointStamped>("/centroidPS_in_robot_base", 1);
    // Approximation point for visualization
    ros::Publisher approx_point_pointStamped = node.advertise<geometry_msgs::PointStamped>("/approximation_point", 1);
    // End- effector position for laser reading for visualization
    ros::Publisher eef_position_laser_reading_pointStamped = node.advertise<geometry_msgs::PointStamped>("/eef_position_laser_reading_point", 1);
    
    tf2_ros::Buffer tfBuffer; 
    // TF listenner
    tf2_ros::TransformListener tfListener(tfBuffer);
    // TF broadcaster to broadcast the TF for the approximation point and for the end-effector position for laser reading
    tf2_ros::TransformBroadcaster br;

    ros::Rate rate(10.0);
    while (node.ok())
    {
        ros::spinOnce();
        tf::StampedTransform transform;
        geometry_msgs::PointStamped centroid_initial_pt;
        centroid_initial_pt.point.x = X;
        centroid_initial_pt.point.y = Y;
        centroid_initial_pt.point.z = Z;
        centroid_initial_pt.header.frame_id = "/camera_rgb_optical_frame";

        geometry_msgs::PointStamped normal_initial_pt;
        normal_initial_pt.point.x = normal_X;
        normal_initial_pt.point.y = normal_Y;
        normal_initial_pt.point.z = normal_Z;
        normal_initial_pt.header.frame_id = "/camera_rgb_optical_frame";

        // APPROXIMATION POINT IN RELATION TO KINECT
        // Calculate approximation point with normal and centroid in relation the TF /camera_rgb_optical_frame 
        geometry_msgs::PointStamped approx_point_initial_pt;
        approx_point_initial_pt.point.x = centroid_initial_pt.point.x + 0.25 * normal_initial_pt.point.x;
        approx_point_initial_pt.point.y = centroid_initial_pt.point.y + 0.25 * normal_initial_pt.point.y;
        approx_point_initial_pt.point.z = centroid_initial_pt.point.z + 0.25 * normal_initial_pt.point.z;

        // Initialize transformation matrixes
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped_laser;
        geometry_msgs::TransformStamped transformStamped_approx_point;
        geometry_msgs::TransformStamped transformStamped_eef_pose;
        geometry_msgs::TransformStamped transformStamped_robot_to_eef;
        try
        {
            // Transformation Matrix from the camera_rgb_optical_frame frame to the robot_base_link frame
            transformStamped = tfBuffer.lookupTransform("robot_base_link", "camera_rgb_optical_frame", ros::Time(0),ros::Duration(3.0));

            geometry_msgs::PointStamped  centroid_transformed_pt; 
            centroid_transformed_pt.header.frame_id = "/robot_base_link";
            geometry_msgs::PointStamped  approx_point_transformed_pt; 
            approx_point_transformed_pt.header.frame_id = "/robot_base_link";

            // Compute the Centroid and Approximation point coordinates in relation to the robot_base_link frame
            tf2::doTransform(centroid_initial_pt, centroid_transformed_pt, transformStamped);
            tf2::doTransform(approx_point_initial_pt, approx_point_transformed_pt, transformStamped);

            geometry_msgs::Vector3 centroid_robot_base;
            centroid_robot_base.x = centroid_transformed_pt.point.x;
            centroid_robot_base.y = centroid_transformed_pt.point.y;
            centroid_robot_base.z = centroid_transformed_pt.point.z;
            // cout << "Centroid: " << centroid_transformed_pt << endl;

            // APPROXIMATION POINT IN RELATION TO ROBOT_BASE_LINK
            geometry_msgs::Vector3 approx_point_robot_base;
            approx_point_robot_base.x = approx_point_transformed_pt.point.x;
            approx_point_robot_base.y = approx_point_transformed_pt.point.y;
            approx_point_robot_base.z = approx_point_transformed_pt.point.z;
            // cout << "Approximation Point : " << approx_point_transformed_pt << endl; 

            // NORMAL IN RELATION TO ROBOT_BASE_LINK
            // Calculate the normal (in relation to the robot_base_link) between the Centroid and the approx Point:
            // the vector is pointing to the Centroid
            geometry_msgs::Vector3 normal_robot_base;
            normal_robot_base.x = centroid_robot_base.x - approx_point_robot_base.x;
            normal_robot_base.y = centroid_robot_base.y - approx_point_robot_base.y;
            normal_robot_base.z = centroid_robot_base.z - approx_point_robot_base.z;

            geometry_msgs::Vector3 normal_robot_base_unit;
            double length;
            length = sqrt ( pow(normal_robot_base.x , 2.0) + pow(normal_robot_base.y , 2.0) + pow(normal_robot_base.z , 2.0) );
            normal_robot_base_unit.x = normal_robot_base.x / length;
            normal_robot_base_unit.y = normal_robot_base.y / length;
            normal_robot_base_unit.z = normal_robot_base.z / length;


            // EULER ANGLES        
            float lengthyz, yaw6, pitch6;

            // yaw - angle in turn of X 
            lengthyz = sqrt ( pow(normal_robot_base_unit.y , 2.0) + pow(normal_robot_base_unit.z , 2.0) );
            yaw6 = atan2(- normal_robot_base_unit.y,normal_robot_base_unit.z) * 180 / M_PI;
            // pitch - angle in turn of Y
            pitch6 = atan2(normal_robot_base_unit.x , lengthyz) * 180 / M_PI;

            float roll, pitch, yaw;
            roll = 0;  
            pitch = pitch6;
            yaw = yaw6;

            cout << "yaw: " << yaw << endl;    
            cout << "pitch: " << -pitch << endl;  

            geometry_msgs::Pose2D  euler_angles;
            euler_angles.x = yaw;
            euler_angles.y = -pitch;

            // END-EFFECTOR POSITION FOR LASER READING
            // Transformation matrix from the eef_tool_tip frame to the ls_optical_frame
            transformStamped_laser = tfBuffer.lookupTransform("ls_optical_frame", "eef_tool_tip", ros::Time(0),ros::Duration(3.0));

            // TF with Origin in the approximation point with the orientation of the Euler angles
            transformStamped_approx_point.header.stamp = ros::Time::now();
            transformStamped_approx_point.header.frame_id = "robot_base_link";
            transformStamped_approx_point.child_frame_id = "approx_point_tf";
            transformStamped_approx_point.transform.translation.x = approx_point_robot_base.x;
            transformStamped_approx_point.transform.translation.y = approx_point_robot_base.y;
            transformStamped_approx_point.transform.translation.z = approx_point_robot_base.z;
            tf2::Quaternion q;
            q.setRPY(-yaw*M_PI/180.0, pitch*M_PI/180.0, M_PI);
            transformStamped_approx_point.transform.rotation.x = q.x();
            transformStamped_approx_point.transform.rotation.y = q.y();
            transformStamped_approx_point.transform.rotation.z = q.z();
            transformStamped_approx_point.transform.rotation.w = q.w();
            br.sendTransform(transformStamped_approx_point);

            // TF with Origin in the end-effector position for laser reading with a translation from the TF in the approximation point
            transformStamped_eef_pose.header.stamp = ros::Time::now();
            transformStamped_eef_pose.header.frame_id = "approx_point_tf";
            transformStamped_eef_pose.child_frame_id = "eef_pose";
            transformStamped_eef_pose.transform.translation.x = transformStamped_laser.transform.translation.x;
            transformStamped_eef_pose.transform.translation.y = transformStamped_laser.transform.translation.y;
            transformStamped_eef_pose.transform.translation.z = transformStamped_laser.transform.translation.z;
            transformStamped_eef_pose.transform.rotation.x = 0;
            transformStamped_eef_pose.transform.rotation.y = 0;
            transformStamped_eef_pose.transform.rotation.z = 0;
            transformStamped_eef_pose.transform.rotation.w = 1;
            br.sendTransform(transformStamped_eef_pose);

            // Transformation from the frame of the end-effector position in relation to the robot_base_link frame
            transformStamped_robot_to_eef = tfBuffer.lookupTransform("robot_base_link", "eef_pose", ros::Time(0),ros::Duration(3.0));

            // End-effector position for laser reading in relation to robot_base_link
            geometry_msgs::Vector3 eef_position_laser_reading;
            eef_position_laser_reading.x = transformStamped_robot_to_eef.transform.translation.x;
            eef_position_laser_reading.y = transformStamped_robot_to_eef.transform.translation.y;
            eef_position_laser_reading.z = transformStamped_robot_to_eef.transform.translation.z;

            // End-effector position PointStamped
            geometry_msgs::PointStamped eef_position_laser_reading_pt;
            eef_position_laser_reading_pt.point.x = eef_position_laser_reading.x;
            eef_position_laser_reading_pt.point.y = eef_position_laser_reading.y;
            eef_position_laser_reading_pt.point.z = eef_position_laser_reading.z;
            eef_position_laser_reading_pt.header.frame_id = "/robot_base_link";

            // MSG to publish Data
            bin_picking::TargetsPose targets_pose;
            targets_pose.header.stamp = ros::Time::now();
            targets_pose.header.frame_id = "/robot_base_link";
            targets_pose.normal = normal_robot_base_unit;
            targets_pose.approx_point = approx_point_robot_base;
            targets_pose.eef_position = eef_position_laser_reading;
            targets_pose.euler_angles = euler_angles;

            //Publish Data:
            targets_pose_pub.publish(targets_pose);

            // For visualization
            centroid_pointStamped.publish(centroid_transformed_pt);
            approx_point_pointStamped.publish(approx_point_transformed_pt);
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