#include "header_pcl.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

/**
* @brief The PassThrough filter is used to identify and/or eliminate points 
within a specific range of X, Y and Z values.
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
within a specific range of X, Y and Z values.
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
void voxelgrid (const PointCloud::ConstPtr& cloud, PointCloud::Ptr cloud_vg_ptr)
{
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.003f, 0.003f, 0.003f); // 5mm
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
  sacs.setDistanceThreshold (0.015);
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
  ror.setMinNeighborsInRadius (15);			
  ror.filter (*cloud_ror_ptr);
}

/**
* @brief Determine the centroid of the input point cloud and its normal
* @param cloud - input cloud
* @param centroid_ptr - point cloud with the center point 
* @param normal_centroid_ptr - normal line 
* @return void
*/
void centroidNormal (const PointCloud::ConstPtr& cloud, PointCloud::Ptr centroid_ptr, pcl::PointCloud<pcl::Normal>::Ptr normal_centroid_ptr)
{
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	
	//  Calculate all the normals of the entire surface
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_surface_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  ne.compute (*cloud_surface_normals_ptr);

	//  The center of the surface
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cloud, centroid);

	//  Index of the center point
	double valor_distancia, valor_distancia_anterior=100;
	int indice_centro=0;
	for (int it_centro=0; it_centro<cloud->points.size (); it_centro++)
	{
	  valor_distancia=abs(cloud->points[it_centro].x-centroid[0])+abs(cloud->points[it_centro].y-centroid[1])+abs(cloud->points[it_centro].z-centroid[2]);    
	  if(valor_distancia<valor_distancia_anterior)
	  {
	    valor_distancia_anterior=valor_distancia;
	    indice_centro=it_centro;
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

	// Normal of the centroid
	normal_centroid_ptr->points.push_back (cloud_surface_normals_ptr->points[indice_centro]);

}

/**
* @brief The EuclideanClusterExtraction ..............EXPLAIN
* @param cloud - input cloud
* @param cloud_pt_ptr - cloud after the application of the filter
* @return void
*/
void euclideanclusterextraction (const PointCloud::ConstPtr& cloud, std::vector<pcl::PointIndices>& ece_indices)
{
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    ece.setClusterTolerance (0.007); //7mm
    // setClusterTolerance()---If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
    // On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster.
    ece.setMinClusterSize (15);
    ece.setMaxClusterSize (30000);
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ece_tree_ptr (new pcl::search::KdTree<pcl::PointXYZRGB>);
    ece_tree_ptr->setInputCloud (cloud);    

    ece.setSearchMethod (ece_tree_ptr);
    ece.setInputCloud (cloud);
    ece.extract (ece_indices);
}

