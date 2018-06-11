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

// for  implementation of a TransformListener to help make the task of receiving transforms easier
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include "message_filters/subscriber.h"

// for visualization
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <cmath>        // std::abs
#include <math.h>       // std::fabs

#include <geometry_msgs/Pose2D.h>
#include <Eigen/Dense>