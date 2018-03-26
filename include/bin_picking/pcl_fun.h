#include "header_pcl.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void passthrough (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_output)
{
  //Remove the points thar are far away from the sensor since thy are points from the ground
  pcl::PassThrough<pcl::PointXYZRGB> pt;
  pt.setInputCloud (cloud);
  pt.setFilterFieldName ("z");
  pt.setFilterLimits (0, 0.9);
  // pcl::PCLPointCloud2::Ptr cloud_pt_kinect_ptr (new pcl::PCLPointCloud2);
  // pcl::PointCloud<pcl::PointXYZ> cloud_pt_kinect_ptr;
//   PointCloud::Ptr cloud_pt_ptr (new PointCloud);
  pt.filter (*cloud_output);
}
