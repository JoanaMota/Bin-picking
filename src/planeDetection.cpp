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

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

/**
* @brief Callback function for processing the subscribed point cloud
* @param cloud_input - cloud to process 
* @return void
*/

void cloud_cb (const PointCloud::ConstPtr& cloud_input)
{
  //Remove the points thar are far away from the sensor since thy are points from the ground
  PointCloud::Ptr cloud_pt_ptr (new PointCloud);
  passthrough(cloud_input, cloud_pt_ptr);

  // Perform the actual filtering
  PointCloud::Ptr cloud_vg_ptr (new PointCloud);
  voxelgrid(cloud_pt_ptr, cloud_vg_ptr);

  // // Identify and remove the table
  PointCloud::Ptr cloud_rest_ptr (new PointCloud);
  sacsegmentation_extindices(cloud_vg_ptr, cloud_rest_ptr);

  // Remove isolated points
  PointCloud::Ptr cloud_filtered (new PointCloud);
  radiusoutlierremoval(cloud_rest_ptr, cloud_filtered);

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