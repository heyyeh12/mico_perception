#include <ros/ros.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/thread/thread.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

ros::Publisher pub0, pub1, pub2;

float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Convert sensor_msgs/PointCloud2 to pcl/PointCloud
  CloudT::Ptr input_pcl (new CloudT),
              cloud_filtered (new CloudT),
              cloud_projected (new CloudT),
              cloud_objects (new CloudT);
  pcl::fromROSMsg(*input, *input_pcl);
  // Do data processing here...

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (input_pcl);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5); // objects between 0 to 1.5 meters away
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  // Find table plane
  pcl::ModelCoefficients::Ptr table_coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr table_indices(new pcl::PointIndices());
  pcl::SACSegmentation<PointT> table_finder;
  table_finder.setOptimizeCoefficients(true);
  table_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  table_finder.setMethodType(pcl::SAC_RANSAC);
  table_finder.setMaxIterations(300);
  table_finder.setAxis(Eigen::Vector3f(1, 0, 0)); // finding horizontal planes
  table_finder.setDistanceThreshold(0.02);
  table_finder.setEpsAngle(deg2rad(5));
  table_finder.setInputCloud(cloud_filtered);
  table_finder.segment(*table_indices, *table_coefficients);

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (table_indices);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (table_coefficients);
  proj.filter (*cloud_projected);
  // std::cerr << "PointCloud after projection has: "
  //           << cloud_projected->points.size () << " data points." << std::endl;

  // Remove the table plane from filtered cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (table_indices);
  extract.setNegative (true);
  extract.filter (*cloud_objects);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_objects);
  //
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_objects);
  ec.extract (cluster_indices);

  // Publish the data.
  pub0.publish (*cloud_filtered);
  pub1.publish(*cloud_projected);
  pub2.publish(*cloud_objects);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segment");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub0 = nh.advertise<sensor_msgs::PointCloud2> ("filtered", 1);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("projected", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("first_object", 1);

  // Control Loop and code
  ros::Rate loop_rate(40.0);
  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
}
