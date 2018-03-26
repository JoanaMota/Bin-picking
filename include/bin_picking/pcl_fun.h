#include "header_pcl.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

/**
* @brief The PassThrough filter is used to identify and/or eliminate points 
within a specific range of X, Y and Z values.
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void passthrough (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_pt_ptr)
{
  //Remove the points thar are far away from the sensor since thy are points from the ground
  pcl::PassThrough<pcl::PointXYZRGB> pt;
  pt.setInputCloud (cloud);
  pt.setFilterFieldName ("z");
  pt.setFilterLimits (0, 0.9);
  pt.filter (*cloud_pt_ptr);
}

/**
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
 with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud - input cloud
* @param cloud_vg_ptr - cloud after the application of the filter
* @return void
*/
void voxelgrid (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_vg_ptr)
{
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.005f, 0.005f, 0.005f);
  vg.filter (*cloud_vg_ptr);
}

/**
* @brief The SACSegmentation and the ExtractIndices filters are used to identify and
remove the table from the point cloud leaving only the objects.
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void sacsegmentation_extindices (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_rest_ptr)
{
  // Identify the table
  pcl::SACSegmentation<pcl::PointXYZRGB> sacs;
  sacs.setOptimizeCoefficients (true);
  sacs.setModelType (pcl::SACMODEL_PLANE);
  sacs.setMethodType (pcl::SAC_RANSAC);
  sacs.setMaxIterations (1000);
  sacs.setDistanceThreshold (0.02);
  sacs.setInputCloud (cloud);
  pcl::PointIndices::Ptr sacs_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr sacs_coefficients (new pcl::ModelCoefficients);
  sacs.segment (*sacs_inliers, *sacs_coefficients);

  // Remove the table
  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud (cloud);
  ei.setIndices (sacs_inliers);
  ei.setNegative (false);
  PointCloud::Ptr cloud_filtered_plan_ptr (new PointCloud);
  ei.filter (*cloud_filtered_plan_ptr);
  ei.setNegative (true);
  ei.filter (*cloud_rest_ptr);
}

/**
* @brief The RadiusOutlierRemoval filter is used to remove isolated point according to
the minimum number of neighbors desired.
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void radiusoutlierremoval (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_ror_ptr)
{
  // Remove isolated points
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
  ror.setInputCloud(cloud);
  ror.setRadiusSearch(0.02);				
  ror.setMinNeighborsInRadius (30);			
  ror.filter (*cloud_ror_ptr);
}



