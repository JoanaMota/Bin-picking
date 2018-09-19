/**
*      @file  planeDetection.c
*      @brief  Program to process a point cloud acquired by the kinect and detect simple objects
*
* Descrição mais detalhada do ficheiros
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

#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

using namespace std;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub_centroid;
ros::Publisher pub_normal;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Objects"));		//Declarar a Visualização

// Vectors to save all the center points and normals
static vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Center;
static vector<pcl::PointCloud<pcl::Normal>::Ptr> Normal;

static int counter;

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

  // IMAGE

  // Func. to perform the actual filtering
  PointCloud::Ptr cloud_vg_ptr (new PointCloud);
  voxelgrid(cloud_ptx_ptr, cloud_vg_ptr, 0.002f);

  // IMAGE

  // Func. to identify and remove the table
  PointCloud::Ptr cloud_rest_ptr (new PointCloud);
  sacsegmentation_extindices(cloud_vg_ptr, cloud_rest_ptr);

  // IMAGE

  // Func. to remove isolated points
  PointCloud::Ptr cloud_filtered (new PointCloud);
  radiusoutlierremoval(cloud_rest_ptr, cloud_filtered);

  // IMAGE

  // Separate the different objects
  vector<pcl::PointIndices> ece_indices;
  euclideanclusterextraction (cloud_filtered, ece_indices);


  // To take frames of the process
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_segmentation(cloud_filtered);
  // viewer->addPointCloud (cloud_filtered, cloud_segmentation, "Segmentation");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Segmentation");

  // To save pcds on files
  pcl::PCDWriter writer;
  Center.clear();
  Normal.clear();
  int objs = 0;
  int r[10] = {255, 0, 0, 255, 255, 0, 255, 153, 153, 255};
  int g[10] = {0, 255, 0, 102, 0, 153, 255, 255, 204, 153};
  int b[10] = {0, 0, 255, 0, 255, 204, 0, 102, 255, 255};
  for (std::vector<pcl::PointIndices>::const_iterator it = ece_indices.begin (); it != ece_indices.end (); ++it)
  {
    //creation of the variables to be replace in each loop
    PointCloud::Ptr cloud_cluster (new PointCloud); 
    PointCloud::Ptr centroid_ptr (new PointCloud);
    PointCloud::Ptr centroid_ptr_real (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr normal_centroid_ptr (new pcl::PointCloud<pcl::Normal>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
  
    // Func. to determine the centroid of the input point cloud and its normal
    centroidNormal(cloud_cluster,centroid_ptr,normal_centroid_ptr,centroid_ptr_real);


    //save the center point
    Center.push_back(centroid_ptr_real);
    Center[0]->header.frame_id = "camera_rgb_optical_frame";
    //save the normal of the center point
    Normal.push_back(normal_centroid_ptr);

    // cout << "Point Cloud " << objs << " has got " << Center[0]->size() << " Points" << endl;

    cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
    // stringstream ss, ss_cloud, ss_centroid, ss_normal, ss_centroid_surf;
    // objs++;
    // ss << "cloud_cluster_" << objs << ".pcd";
    // ss_cloud << "cloud_cluster_" << objs;
    // ss_centroid << "cloud_centroid_" << objs;
    // ss_centroid_surf << "cloud_centroid_surf_" << objs;
    // ss_normal << "cloud_normal_" << objs;
    // string pointcloudName = ss_cloud.str();
    // string pointcloudCentroid = ss_centroid.str();
    // string pointcloudCentroidSurf = ss_centroid_surf.str();
    // string pointcloudNormal = ss_normal.str();
    // // writer.write<pcl::PointXYZRGB> ("cloud_cluster_1.pcd", *Center[0], false); //*

    // // IMAGE
      
    // // Visualization of the point cloud with the objects SEPARATE BY COLORS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler (cloud_cluster,r[objs-1],g[objs-1],b[objs-1]);
    // // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> surface_handler (cloud_cluster);
    // viewer->addPointCloud (cloud_cluster, surface_handler, pointcloudName);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pointcloudName);
    // cout << pointcloudName << endl; 

    // // Visualization of the centroid point of every object
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler_centroid (centroid_ptr_real,r[objs-1],g[objs-1],b[objs-1]);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler_centroid (centroid_ptr_real,255,0,0);
    // viewer->addPointCloud (centroid_ptr_real, surface_handler_centroid, pointcloudCentroid);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, pointcloudCentroid);

    // // Visualization of the centroid point of every object
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> surface_handler_centroid_surf (centroid_ptr,255,255,255);
    // viewer->addPointCloud (centroid_ptr, surface_handler_centroid_surf, pointcloudCentroidSurf);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, pointcloudCentroidSurf);
    
    // // Visualization of the the normal on the centroid of every object
    // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (centroid_ptr_real, normal_centroid_ptr,100,0.05, pointcloudNormal);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, pointcloudNormal);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 1, pointcloudNormal);


  }
  // cout << Center[4]->points[0].x*1000 << endl;
  // cout << Center[0]->points[0].x*1000 << endl;
  // cout << "Callback" << endl;
  counter++;
  // Publish the data
    pub.publish (cloud_filtered);
    pub_centroid.publish (Center[0]);
    pub_normal.publish (Normal[0]);
  // pub2.publish (cloud_rest_ptr);
}


int main (int argc, char** argv)
{
  // viewer->setBackgroundColor (0.2, 0.2, 0.2);
  // viewer->addCoordinateSystem (0.1,0,0,0);
  // setCameraPosition (double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z, double up_x, double up_y, double up_z, int viewport=0
  // viewer->setCameraPosition (1, 0,-1, 1, -2, 1, 0, -1, 0);
  
  // Initialize ROS
  ros::init (argc, argv, "bin_picking_objDetection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<PointCloud> ("/camera/depth_registered/points", 1, cloud_cb);


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("/output_kinect", 1);
  // pub2 = nh.advertise<PointCloud> ("/output_kinect_before_isolated_points", 1);
  pub_centroid = nh.advertise<PointCloud> ("/cloud_centroid", 1);
  pub_normal = nh.advertise<PointCloud> ("/cloud_centroid_normal", 1);
  
  // char keyInput;
  ros::Rate r(100);
  while(nh.ok())
  {  
    // keyInput = getch();
    
    // if(keyInput == 's' || keyInput == 'S')

    // if(counter == 10)
    // {
    //   break;
    // }
    ros::spinOnce();
    // viewer->spinOnce();

    // pub_centroid.publish (Center[0]);
    // pub_normal.publish (Normal[0]);
  }
  // cout << "Number of clusters: " << Center.size() << endl;
  // while (nh.ok())
  // {
  //   pub_centroid.publish (Center[0]);
  //   pub_normal.publish (Normal[0]);
  // }

  // Spin
  // ros::spin ();
}