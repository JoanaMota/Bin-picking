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

    // cout << "X: " << X << endl;
    // cout << "Y: " << Y << endl;
    // cout << "Z: " << Z << endl;
}

void cloud_centroid_normal (const PointCloudNormal::ConstPtr& cloud_normal)
{
    normal_X = cloud_normal->points[0].normal_x;
    normal_Y = cloud_normal->points[0].normal_y;
    normal_Z = cloud_normal->points[0].normal_z;

    // cout << "W: " << normal_X << endl;
    // cout << "P: " << normal_Y << endl;
    // cout << "R: " << normal_Z << endl;
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
    // //-Centroid in global frame(robot_base_link)
    // ros::Publisher centroid_pub = node.advertise<geometry_msgs::Vector3>("/centroid_in_robot_base", 1);
    //-Normal in global frame(robot_base_link)
    // ros::Publisher normal_pub = node.advertise<geometry_msgs::Vector3>("/normal_in_robot_base", 1);
    // //-Approximation Point in global frame(robot_base_link)
    // ros::Publisher approx_point_pub = node.advertise<geometry_msgs::Vector3>("/approximation_point_in_robot_base", 1);
    // //-End-effector position for laser reading in global frame(robot_base_link)
    // ros::Publisher eef_position_laser_reading_pub = node.advertise<geometry_msgs::Vector3>("/eef_position_laser_reading_in_robot_base", 1);
    // //Euler angles
    // ros::Publisher euler_angles_pub = node.advertise<geometry_msgs::Pose2D>("/euler_angles", 1);
    ros::Publisher targets_pose_pub = node.advertise<bin_picking::TargetsPose>("/targets_pose", 1);

    //For visualization
    // Centroid for visualization
    ros::Publisher centroid_pointStamped = node.advertise<geometry_msgs::PointStamped>("/centroidPS_in_robot_base", 1);
    // Approximation point for visualization
    ros::Publisher approx_point_pointStamped = node.advertise<geometry_msgs::PointStamped>("/approximation_point", 1);
    // End- effector position for laser reading visualitation
    ros::Publisher eef_position_laser_reading_pointStamped = node.advertise<geometry_msgs::PointStamped>("/eef_position_laser_reading_point", 1);
    // ros::Publisher centroid_pointStamped_depth = node.advertise<geometry_msgs::PointStamped>("/centroidPS_in_robot_base_depth", 1);
    tf2_ros::Buffer tfBuffer; 
    tf2_ros::TransformListener tfListener(tfBuffer);

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
        // geometry_msgs::PointStamped centroid_initial_depth_pt;
        // centroid_initial_depth_pt.point.x = X;
        // centroid_initial_depth_pt.point.y = Y;
        // centroid_initial_depth_pt.point.z = Z;
        // centroid_initial_depth_pt.header.frame_id = "/camera_depth_optical_frame";

            // cout << "centroid_initial_pt_x: " << centroid_initial_pt.point.x << endl; 
            // cout << "centroid_initial_pt_y: " << centroid_initial_pt.point.y << endl; 
            // cout << "centroid_initial_pt_z: " << centroid_initial_pt.point.z << endl; 

        geometry_msgs::PointStamped normal_initial_pt;
        normal_initial_pt.point.x = normal_X;
        normal_initial_pt.point.y = normal_Y;
        normal_initial_pt.point.z = normal_Z;
        normal_initial_pt.header.frame_id = "/camera_rgb_optical_frame";


        // Calculate approximation point with normal and centroid
        geometry_msgs::PointStamped approx_point_initial_pt;
        approx_point_initial_pt.point.x = centroid_initial_pt.point.x + 0.15 * normal_initial_pt.point.x;
        approx_point_initial_pt.point.y = centroid_initial_pt.point.y + 0.15 * normal_initial_pt.point.y;
        approx_point_initial_pt.point.z = centroid_initial_pt.point.z + 0.15 * normal_initial_pt.point.z;


        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped_laser;
        try
        {
            transformStamped = tfBuffer.lookupTransform("robot_base_link", "camera_rgb_optical_frame", ros::Time(0),ros::Duration(3.0));

            // cout << "transformation" << transformStamped << endl; 

            geometry_msgs::PointStamped  centroid_transformed_pt; 
            centroid_transformed_pt.header.frame_id = "/robot_base_link";
            geometry_msgs::PointStamped  approx_point_transformed_pt; 
            approx_point_transformed_pt.header.frame_id = "/robot_base_link";

            // Compute the Centroid and Approximation point coordinates in relation to the robot_base_link frame
            tf2::doTransform(centroid_initial_pt, centroid_transformed_pt, transformStamped);
            tf2::doTransform(approx_point_initial_pt, approx_point_transformed_pt, transformStamped);

            // approx_point_pt = centroid_transformed_pt + 0.05 * normal_transformed_pt;

            geometry_msgs::Vector3 centroid_robot_base;
            centroid_robot_base.x = centroid_transformed_pt.point.x;
            centroid_robot_base.y = centroid_transformed_pt.point.y;
            centroid_robot_base.z = centroid_transformed_pt.point.z;

            cout << "Centroid: " << centroid_transformed_pt << endl;
            
            geometry_msgs::Vector3 approx_point_robot_base;
            approx_point_robot_base.x = approx_point_transformed_pt.point.x;
            approx_point_robot_base.y = approx_point_transformed_pt.point.y;
            approx_point_robot_base.z = approx_point_transformed_pt.point.z;

            // cout << "Approximation Point : " << approx_point_transformed_pt << endl; 

            // Calculate the normal (in relation to the robot_base_link) between the the Centroid and the approx Point:
            // This is already an unit vector
            geometry_msgs::Vector3 normal_robot_base;
            normal_robot_base.x = centroid_robot_base.x - approx_point_robot_base.x;
            normal_robot_base.y = centroid_robot_base.y - approx_point_robot_base.y;
            normal_robot_base.z = centroid_robot_base.z - approx_point_robot_base.z;
            // normal_robot_base.x = approx_point_robot_base.x - centroid_robot_base.x;
            // normal_robot_base.y = approx_point_robot_base.y - centroid_robot_base.y;
            // normal_robot_base.z = approx_point_robot_base.z - centroid_robot_base.z;

            geometry_msgs::Vector3 normal_robot_base_unit;
            double length;
            length = sqrt ( pow(normal_robot_base.x , 2.0) + pow(normal_robot_base.y , 2.0) + pow(normal_robot_base.z , 2.0) );
            normal_robot_base_unit.x = normal_robot_base.x / length;
            normal_robot_base_unit.y = normal_robot_base.y / length;
            normal_robot_base_unit.z = normal_robot_base.z / length;

            // cout << "normal in robot_base: " << normal_robot_base_unit << endl; 

            // EULER ANGLES        

            // float yaw, pitch, lengthvec, pitch2, lengthyz, lengthxz, yaw3, pitch3;
            float yaw3, pitch3, lengthyz, lengthxz, yaw_final, pitch_final, yaw4, pitch4, lengthxy, yaw5, pitch5, yaw6, pitch6, lengthxyz;

            // yaw = atan2 ( normal_robot_base_unit.y , normal_robot_base_unit.z ) * 180 / M_PI;
            // pitch = atan2 ( normal_robot_base_unit.x , normal_robot_base_unit.z ) * 180 / M_PI;
            
            // // lengthvec = sqrt ( pow(normal_X , 2.0) + pow(normal_Y , 2.0) + pow(normal_Z , 2.0) );
            // lengthvec = sqrt ( pow(normal_robot_base_unit.x , 2.0) + pow(normal_robot_base_unit.z , 2.0) );
            // pitch2 = acos(normal_robot_base_unit.z / lengthvec);

            // // pitch - angle in turn of Y
            // lengthxz = sqrt ( pow(normal_robot_base_unit.x , 2.0) + pow(normal_robot_base_unit.z , 2.0) );
            // pitch3 = asin(normal_robot_base_unit.x / lengthxz ) * 180 / M_PI;
            // pitch_final = abs(pitch3);

            // yaw3 = asin(-normal_robot_base_unit.y / lengthxz ) * 180 / M_PI; 
            // yaw_final = - abs( yaw3 + 180 );

            // // yaw - angle in turn of X 
            // yaw4 = atan2(normal_robot_base_unit.y,normal_robot_base_unit.x) * 180 / M_PI;
            // pitch4 = acos(normal_robot_base_unit.z) * 180 / M_PI;

            // lengthxy = sqrt ( pow(normal_robot_base_unit.x , 2.0) + pow(normal_robot_base_unit.y , 2.0) );
            // yaw5 = atan2(normal_robot_base_unit.y,normal_robot_base_unit.x) * 180 / M_PI;
            // pitch5 = atan2(lengthxy , normal_robot_base_unit.z) * 180 / M_PI;

            // lengthxyz = sqrt ( pow(normal_robot_base_unit.x , 2.0) + pow(normal_robot_base_unit.y , 2.0) + pow(normal_robot_base_unit.z , 2.0) );
            // yaw - angle in turn of X 
            lengthyz = sqrt ( pow(normal_robot_base_unit.y , 2.0) + pow(normal_robot_base_unit.z , 2.0) );
            yaw6 = atan2(- normal_robot_base_unit.y,normal_robot_base_unit.z) * 180 / M_PI;
            // pitch - angle in turn of Y
            pitch6 = atan2(normal_robot_base_unit.x , lengthyz) * 180 / M_PI;

            // cout << "yaw_final: " << yaw_final << endl;    
            // cout << "pitch_final: " << pitch_final << endl;    

            float roll, pitch, yaw;
            roll = 0;  
            pitch = pitch6;
            yaw = yaw6;

            cout << "yaw: " << yaw << endl;    
            cout << "pitch: " << pitch << endl;  

            geometry_msgs::Pose2D  euler_angles;
            euler_angles.x=yaw;
            euler_angles.y=abs(pitch);

            // APPROXIMATION POINT FOR LASER
            transformStamped_laser = tfBuffer.lookupTransform("ls_optical_frame", "eef_tool_tip", ros::Time(0),ros::Duration(3.0));
            // cout << "Transformation from tool tip to laser" << transformStamped_laser << endl;

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
            T_eef_ls(0,3) = transformStamped_laser.transform.translation.x;
            T_eef_ls(1,3) = transformStamped_laser.transform.translation.y;
            T_eef_ls(2,3) = transformStamped_laser.transform.translation.z;
            T_eef_ls(3,0) = 0;
            T_eef_ls(3,1) = 0;
            T_eef_ls(3,2) = 0;
            T_eef_ls(3,3) = 1;

            // cout << "Transformation from enf-effector to laser: \n" << T_eef_ls << endl; 
            
            Eigen::Matrix4f T_robot_point(4,4);
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

            // cout << "Transformation from robot base to End-effector: \n" << T_robot_eef << endl;

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

            //Publish Data:
            // centroid_pub.publish(centroid_robot_base);
            // normal_pub.publish(normal_robot_base_unit);
            // approx_point_pub.publish(approx_point_robot_base);
            // euler_angles_pub.publish(euler_angles);
            // eef_position_laser_reading_pub.publish(eef_position_laser_reading);

            bin_picking::TargetsPose targets_pose;
            targets_pose.header.stamp = ros::Time::now();
            targets_pose.header.frame_id = "/robot_base_link";
            targets_pose.normal = normal_robot_base_unit;
            targets_pose.approx_point = approx_point_robot_base;
            targets_pose.eef_position = eef_position_laser_reading;
            targets_pose.euler_angles = euler_angles;

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