#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  //Restringir área de trabalho 
  pcl::PassThrough<pcl::PCLPointCloud2> pt;

  pt.setInputCloud (cloudPtr);
  pt.setFilterFieldName ("z");
  pt.setFilterLimits (0, 0.7);
  pcl::PCLPointCloud2::Ptr cloud_pt_kinect_ptr1 (new pcl::PCLPointCloud2);
  pt.filter (*cloud_pt_kinect_ptr1);

  pt.setInputCloud (cloud_pt_kinect_ptr1);
  pt.setFilterFieldName ("y");
  pt.setFilterLimits (-0.50, 0.10);
  pcl::PCLPointCloud2::Ptr cloud_pt_kinect_ptr2 (new pcl::PCLPointCloud2);
  pt.filter (*cloud_pt_kinect_ptr2);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_pt_kinect_ptr2);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "bin_picking");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/output_kinect", 1);

  // Spin
  ros::spin ();
}