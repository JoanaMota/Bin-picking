/**
*      @file  planeDetection.c
*      @brief  Program to process a point cloud acquired by the kinect and detect simple objects
*
* Descrição mais detalhada do ficheiro
*
*     @author  Joana Mota, joanacarvalhomota@ua.pt
*
*   @internal
*     Created  21-Mar-2018
*     Company  University of Aveiro
*   Copyright  Copyright (c) 2017, Joana Mota
*
***************************************************
*/

#include "../include/bin_picking/header_pcl.h"
#include "../include/bin_picking/pcl_fun.h"

using namespace std;

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Objetos"));		//Declarar a Visualização


/**
* @brief Callback function for processing the subscribed point cloud
* @param cloud_input - cloud to process 
* @return void
*/

void cloud_cb (const PointCloud::ConstPtr& cloud_input)
{
  // Func. to remove the points thar are far away from the sensor since thy are points from the ground
  PointCloud::Ptr cloud_ptz_ptr (new PointCloud);
  passthroughZ(cloud_input, cloud_ptz_ptr);

  PointCloud::Ptr cloud_ptx_ptr (new PointCloud);
  passthroughX(cloud_ptz_ptr, cloud_ptx_ptr,-0.20,0.60);

  // Func. to perform the actual filtering
  PointCloud::Ptr cloud_vg_ptr (new PointCloud);
  voxelgrid(cloud_ptx_ptr, cloud_vg_ptr);

  // Func. to identify and remove the table
  PointCloud::Ptr cloud_rest_ptr (new PointCloud);
  sacsegmentation_extindices(cloud_vg_ptr, cloud_rest_ptr);

  // Func. to remove isolated points
  PointCloud::Ptr cloud_filtered (new PointCloud);
  radiusoutlierremoval(cloud_rest_ptr, cloud_filtered);

  // Func. to determine the centroid of the input point cloud and its normal
  PointCloud::Ptr centroid_ptr (new PointCloud);
	pcl::PointCloud<pcl::Normal>::Ptr normal_centroid_ptr (new pcl::PointCloud<pcl::Normal>);
  centroidNormal(cloud_filtered,centroid_ptr,normal_centroid_ptr);

	// Visualization of the point cloud with only the box
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler (cloud_filtered,128,255,0);
	viewer->addPointCloud (cloud_filtered, surface_handler, "Point Cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Point Cloud");

  // Visualization of the centroid point
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler_centroid (centroid_ptr,51,51,250);
	viewer->addPointCloud (centroid_ptr, surface_handler_centroid, "Point Cloud centroid");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Point Cloud centroid");
  
  // Visualization of the the normal on the centroid
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (centroid_ptr, normal_centroid_ptr,100,0.1, "Point Normals");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "Point Normals");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 1, "Point Normals"); 
	
  viewer->spinOnce();
  // Publish the data
  pub.publish (cloud_filtered);
}


int main (int argc, char** argv)
{
  viewer->setBackgroundColor (0.3, 0.3, 0.3);
  viewer->addCoordinateSystem (0.1,0,0,0);

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