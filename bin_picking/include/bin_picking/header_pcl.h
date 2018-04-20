#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

//for the typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud
#include <pcl_ros/point_cloud.h>



// #include <pcl/registration/ia_ransac.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

//for the euclideanclusterextraction
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
