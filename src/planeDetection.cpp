
#include "../include/bin_picking/header_pcl.h"
#include "../include/bin_picking/pcl_fun.h"

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void cloud_cb (const PointCloud::ConstPtr& cloud_input)
{
  // // Container for original & filtered data
  // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;

  // // Convert to PCL data type
  // pcl_conversions::toPCL(*cloud_msg, *cloud);

  // //Remove the points thar are far away from the sensor since thy are points from the ground
  // pcl::PassThrough<pcl::PointXYZRGB> pt;
  // pt.setInputCloud (cloud_input);
  // pt.setFilterFieldName ("z");
  // pt.setFilterLimits (0, 0.9);
  // // pcl::PCLPointCloud2::Ptr cloud_pt_kinect_ptr (new pcl::PCLPointCloud2);
  // // pcl::PointCloud<pcl::PointXYZ> cloud_pt_kinect_ptr;
  PointCloud::Ptr cloud_pt_ptr (new PointCloud);
  // pt.filter (*cloud_pt_ptr);
  passthrough(cloud_input, cloud_pt_ptr);

  // pt.setInputCloud (cloud_pt_kinect_ptr1);
  // pt.setFilterFieldName ("y");
  // pt.setFilterLimits (-0.50, 0.10);
  // pcl::PCLPointCloud2::Ptr cloud_pt_kinect_ptr2 (new pcl::PCLPointCloud2);
  // pt.filter (*cloud_pt_kinect_ptr2);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_pt_ptr);
  vg.setLeafSize (0.005f, 0.005f, 0.005f);
  PointCloud::Ptr cloud_vg_ptr (new PointCloud);
  // pcl::PCLPointCloud2::Ptr cloud_vg_kinect_ptr (new pcl::PCLPointCloud2);  
  vg.filter (*cloud_vg_ptr);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::fromPCLPointCloud2 (*cloud_vg_kinect_ptr, *cloud_ptr);

  // Identify the table
  pcl::SACSegmentation<pcl::PointXYZRGB> sacs;
  sacs.setOptimizeCoefficients (true);
  sacs.setModelType (pcl::SACMODEL_PLANE);
  sacs.setMethodType (pcl::SAC_RANSAC);
  sacs.setMaxIterations (1000);
  sacs.setDistanceThreshold (0.02);
  sacs.setInputCloud (cloud_vg_ptr);
  pcl::PointIndices::Ptr sacs_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr sacs_coefficients (new pcl::ModelCoefficients);
  sacs.segment (*sacs_inliers, *sacs_coefficients);

  // Remove the table
  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud (cloud_vg_ptr);
  ei.setIndices (sacs_inliers);
  ei.setNegative (false);
  PointCloud::Ptr cloud_filtered_plan_ptr (new PointCloud);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_plan_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  ei.filter (*cloud_filtered_plan_ptr);
  ei.setNegative (true);
  PointCloud::Ptr cloud_rest_ptr (new PointCloud);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rest_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  ei.filter (*cloud_rest_ptr);
  
  // Remove isolated points
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
  ror.setInputCloud(cloud_rest_ptr);
  ror.setRadiusSearch(0.02);				///////////////////
  ror.setMinNeighborsInRadius (30);			///////////////////
  PointCloud::Ptr cloud_filtered (new PointCloud);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  ror.filter (*cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "bin_picking");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<PointCloud> ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("/output_kinect", 1);

  // Spin
  ros::spin ();
}