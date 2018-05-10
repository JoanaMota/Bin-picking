#include "../../bin_picking/include/bin_picking/header_pcl.h"

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

    ros::Publisher centroid_pub = node.advertise<geometry_msgs::Vector3>("/centroid_in_robot_base", 1);
    ros::Publisher normal_pub = node.advertise<geometry_msgs::Vector3>("/normal_in_robot_base", 1);
    ros::Publisher centroid_pointStamped = node.advertise<geometry_msgs::PointStamped>("/centroidPS_in_robot_base", 1);
    ros::Publisher centroid_pointStamped_depth = node.advertise<geometry_msgs::PointStamped>("/centroidPS_in_robot_base_depth", 1);
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
        geometry_msgs::PointStamped centroid_initial_depth_pt;
        centroid_initial_depth_pt.point.x = X;
        centroid_initial_depth_pt.point.y = Y;
        centroid_initial_depth_pt.point.z = Z;
        centroid_initial_depth_pt.header.frame_id = "/camera_depth_optical_frame";

            cout << "centroid_initial_pt_x: " << centroid_initial_pt.point.x << endl; 
            cout << "centroid_initial_pt_y: " << centroid_initial_pt.point.y << endl; 
            cout << "centroid_initial_pt_z: " << centroid_initial_pt.point.z << endl; 

        geometry_msgs::PointStamped normal_initial_pt;
        normal_initial_pt.point.x = normal_X;
        normal_initial_pt.point.y = normal_Y;
        normal_initial_pt.point.z = normal_Z;
        normal_initial_pt.header.frame_id = "/camera_rgb_optical_frame";

        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped_depth;
        geometry_msgs::TransformStamped transformStamped_rgb;
        try
        {
            // ATENÇAO EXPLICAR QUAL É O REFERENCIAL DA CAMERA
            // void Transformer::lookupTransform 	(const std::string & "target_frame",const std::string & "source_frame",
            // const ros::Time & time)

            // transformStamped_depth = tfBuffer.lookupTransform("camera_depth_optical_frame", "camera_rgb_optical_frame", ros::Time(0),ros::Duration(3.0));
            // cout << "trans rgb to depth \n" << transformStamped_depth << endl; 
            // geometry_msgs::PointStamped  centroid_transformed_depth_pt; 
            // centroid_transformed_depth_pt.header.frame_id = "/camera_depth_optical_frame";            
            // tf2::doTransform(centroid_initial_pt, centroid_transformed_depth_pt, transformStamped_depth);
            
            // cout << "point in DEPTH" << endl;
            // cout << centroid_transformed_depth_pt << endl;

            transformStamped = tfBuffer.lookupTransform("robot_base_link", "camera_depth_optical_frame", ros::Time(0),ros::Duration(3.0));
            geometry_msgs::PointStamped  centroid_transformed_depth_pt; 
            centroid_transformed_depth_pt.header.frame_id = "/robot_base_link";
            tf2::doTransform(centroid_initial_depth_pt, centroid_transformed_depth_pt, transformStamped);

            
            cout << "point in ROBOT from DEPTH" << endl;
            cout << centroid_transformed_depth_pt << endl;

            centroid_pointStamped_depth.publish(centroid_transformed_depth_pt);


            transformStamped_rgb = tfBuffer.lookupTransform("robot_base_link", "camera_rgb_optical_frame", ros::Time(0),ros::Duration(3.0));

            geometry_msgs::PointStamped  centroid_transformed_pt; 
            centroid_transformed_pt.header.frame_id = "/robot_base_link";

            tf2::doTransform(centroid_initial_pt, centroid_transformed_pt, transformStamped_rgb);
            
            geometry_msgs::Vector3 centroid_robot_base;
            centroid_robot_base.x = centroid_transformed_pt.point.x;
            centroid_robot_base.y = centroid_transformed_pt.point.y;
            centroid_robot_base.z = centroid_transformed_pt.point.z;

            cout << "point in ROBOT from RGB" << endl;
            cout << centroid_transformed_pt << endl;

            

//------------------------------------------------------------------------------------------------------------------------------------------------
            // transformStamped = tfBuffer.lookupTransform("robot_base_link", "camera_rgb_optical_frame", ros::Time(0),ros::Duration(3.0));

            // cout << "transformation" << transformStamped << endl; 

            // geometry_msgs::PointStamped  centroid_transformed_pt; 
            // centroid_transformed_pt.header.frame_id = "/robot_base_link";
            // geometry_msgs::PointStamped  normal_transformed_pt; 
            // normal_transformed_pt.header.frame_id = "/robot_base_link";

            // tf2::doTransform(centroid_initial_pt, centroid_transformed_pt, transformStamped);
            // tf2::doTransform(normal_initial_pt, normal_transformed_pt, transformStamped);
            
            // geometry_msgs::Vector3 centroid_robot_base;
            // centroid_robot_base.x = centroid_transformed_pt.point.x;
            // centroid_robot_base.y = centroid_transformed_pt.point.y;
            // centroid_robot_base.z = centroid_transformed_pt.point.z;

            // cout << "centroid_robot_base_x: " << centroid_robot_base.x << endl; 
            // cout << "centroid_robot_base_y: " << centroid_robot_base.y << endl; 
            // cout << "centroid_robot_base_z: " << centroid_robot_base.z << endl; 
            
            // geometry_msgs::Vector3 normal_robot_base;
            // normal_robot_base.x = normal_transformed_pt.point.x;
            // normal_robot_base.y = normal_transformed_pt.point.y;
            // normal_robot_base.z = normal_transformed_pt.point.z;

            centroid_pub.publish(centroid_robot_base);
            // normal_pub.publish(normal_robot_base);
            centroid_pointStamped.publish(centroid_transformed_pt);
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