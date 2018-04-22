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
ros::Publisher pub2;
ros::Publisher pub_centroid;
ros::Publisher pub_normal;

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

  // Func. to remove the base of the robot
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

  // Separate the diffrent objs
  vector<pcl::PointIndices> ece_indices;
  euclideanclusterextraction (cloud_filtered, ece_indices);
  
  // To save pcds on files
  pcl::PCDWriter writer;
  // Vectors to save all the center points and normals
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Center;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> Normal;

  int objs = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = ece_indices.begin (); it != ece_indices.end (); ++it)
  {
    PointCloud::Ptr centroid_ptr (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr normal_centroid_ptr (new pcl::PointCloud<pcl::Normal>);
    PointCloud::Ptr cloud_cluster (new PointCloud); 
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
  
    // Func. to determine the centroid of the input point cloud and its normal
    centroidNormal(cloud_cluster,centroid_ptr,normal_centroid_ptr);
    //save the center point
    Center.push_back(centroid_ptr);
    //save the normal of the center point
    Normal.push_back(normal_centroid_ptr);

    cout << "Point Cloud " << objs << " has got " << Center[0]->size() << " Points" << endl;

    cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
    stringstream ss_cloud, ss_centroid, ss_normal;
    objs++;
    ss_cloud << "cloud_cluster_" << objs;
    ss_centroid << "cloud_centroid_" << objs;
    ss_normal << "cloud_normal_" << objs;
    string pointcloudName = ss_cloud.str();
    string pointcloudCentroid = ss_centroid.str();
    string pointcloudNormal = ss_normal.str();
    // writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
      
    // Visualization of the point cloud with the objects
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler (cloud_cluster,255,255,0);
    viewer->addPointCloud (cloud_cluster, surface_handler, pointcloudName);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pointcloudName);

    // Visualization of the centroid point of every object
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler_centroid (centroid_ptr,0,51,250);
    viewer->addPointCloud (centroid_ptr, surface_handler_centroid, pointcloudCentroid);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, pointcloudCentroid);
    
    // Visualization of the the normal on the centroid of every object
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (centroid_ptr, normal_centroid_ptr,100,0.1, pointcloudNormal);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, pointcloudNormal);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 1, pointcloudNormal); 
    
  }
  cout << Center[4]->points[0].x*1000 << endl;
  cout << Center[0]->points[0].x*1000 << endl;
	
  viewer->spinOnce();

  // Publish the data
  pub.publish (cloud_filtered);
  pub2.publish (cloud_rest_ptr);
  pub_centroid.publish (Center[0]);
  pub_normal.publish (Normal[0]);
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
  pub2 = nh.advertise<PointCloud> ("/output_kinect_before_isolated_points", 1);
  pub_centroid = nh.advertise<PointCloud> ("/cloud_centroid", 1);
  pub_normal = nh.advertise<PointCloud> ("/cloud_centroid_normal", 1);

  // Spin
  ros::spin ();
}