#include "header_pcl.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

/**
* @brief The PassThrough filter is used to identify and/or eliminate points 
within a specific range of X, Y and Z values. In this case in the Z direction
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void passthroughZ (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_pt_ptr)
{
  //Remove the points thar are far away from the sensor since thy are points from the ground
  pcl::PassThrough<pcl::PointXYZRGB> pt;
  pt.setInputCloud (cloud);
  pt.setFilterFieldName ("z");
  pt.setFilterLimits (0, 0.9);
  pt.filter (*cloud_pt_ptr);
}

/**
* @brief The PassThrough filter is used to identify and/or eliminate points 
within a specific range of X, Y and Z values. In this case in the X direction
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void passthroughX (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_pt_ptr, float min, float max)
{
  //Remove the points thar are far away from the sensor since thy are points from the ground
  pcl::PassThrough<pcl::PointXYZRGB> pt;
  pt.setInputCloud (cloud);
  pt.setFilterFieldName ("x");
  pt.setFilterLimits (min, max);
  pt.filter (*cloud_pt_ptr);
}

/**
* @brief The VoxelGrid filter is used to simplify the cloud, by wrapping the point cloud
 with a three-dimensional grid and reducing the number of points to the center points within each bloc of the grid.
* @param cloud - input cloud
* @param cloud_vg_ptr - cloud after the application of the filter
* @return void
*/
void voxelgrid (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_vg_ptr, float gridside)
{
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (gridside, gridside, gridside); // 3mm float variable
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
  sacs.setDistanceThreshold (0.0155);
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
  ror.setMinNeighborsInRadius (45);			
  ror.filter (*cloud_ror_ptr);
}

/**
* @brief Determine the centroid of the input point cloud and its normal
* @param cloud - input cloud
* @param centroid_ptr - point cloud with the center point 
* @param normal_centroid_ptr - normal line 
* @return void
*/
void centroidNormal (const PointCloud::ConstPtr& cloud, PointCloud::Ptr centroid_ptr, pcl::PointCloud<pcl::Normal>::Ptr normal_centroid_ptr, PointCloud::Ptr centroid_ptr_real)
{
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	
	//  Calculate all the normals of the entire surface
  ne.setInputCloud (cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  // ne.setKSearch (50);
  // Use all neighbors in a sphere of radius 35mm
  ne.setRadiusSearch (0.05);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_surface_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*cloud_surface_normals_ptr);

	//  The center of the surface
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cloud, centroid);

	//  Index of the center point
	double distance, distance_before=1000;
	int index_center=0;
	for (int it_center=0; it_center<cloud->points.size (); it_center++)
	{
	  distance = fabs(cloud->points[it_center].x-centroid[0])   +    fabs(cloud->points[it_center].y-centroid[1])   +    fabs(cloud->points[it_center].z-centroid[2]); 
	  if(distance < distance_before)
	  {
	    distance_before=distance;
	    index_center=it_center;
	  }
	} 
	
	// The center point
  centroid_ptr->points.push_back (cloud->points[1]);
  centroid_ptr->width = centroid_ptr->points.size ();
  centroid_ptr->height = 1;
  centroid_ptr->is_dense = true;

  centroid_ptr->points[0].x=centroid[0];
  centroid_ptr->points[0].y=centroid[1];
  centroid_ptr->points[0].z=centroid[2];

	// The Real center point
  centroid_ptr_real->points.push_back (cloud->points[1]);
  centroid_ptr_real->width = centroid_ptr_real->points.size ();
  centroid_ptr_real->height = 1;
  centroid_ptr_real->is_dense = true;

  centroid_ptr_real->points[0].x=cloud->points[index_center].x;
  centroid_ptr_real->points[0].y=cloud->points[index_center].y;
  centroid_ptr_real->points[0].z=cloud->points[index_center].z;

	// Normal of the centroid
	normal_centroid_ptr->points.push_back (cloud_surface_normals_ptr->points[index_center]);

}

/**
* @brief The EuclideanClusterExtraction ..............EXPLAIN
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void euclideanclusterextraction (const PointCloud::ConstPtr& cloud, std::vector<pcl::PointIndices>& ece_indices)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ece_tree_ptr (new pcl::search::KdTree<pcl::PointXYZRGB>);
    ece_tree_ptr->setInputCloud (cloud);    

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    ece.setClusterTolerance (0.035); //20mm
    // setClusterTolerance()---If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
    // On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster.
    ece.setMinClusterSize (100);
    ece.setMaxClusterSize (2500);
    // ece.setMinClusterSize (15);
    // ece.setMaxClusterSize (30000);
    ece.setSearchMethod (ece_tree_ptr);
    ece.setInputCloud (cloud);
    ece.extract (ece_indices);
}

