#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  cloudptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloudptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *point_cloud);
  sensor_msgs::PointCloud2 cloud_filtered;

  pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter (true);
  cropBoxFilter.setInputCloud(point_cloud);
  Eigen::Vector4f min_pt (-0.2f,-0.1f,-1.0f,1.0f);
  Eigen::Vector4f max_pt (0.3f,0.3f,1.0f,1.0f);

  cropBoxFilter.setMin(min_pt);
  cropBoxFilter.setMax(max_pt);

//   vector<int> indices;
//   cropBoxFilter.filter(indices);

  cropBoxFilter.filter(*filtered_point_cloud);
//   cropBoxFilter.


//   // Voxel Grid Filter 

//   pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//   sor.setInputCloud(point_cloud);
//   sor.setLeafSize(0.05, 0.05, 0.05);
//   sor.filter(*filtered_point_cloud);

  pcl::toROSMsg(*filtered_point_cloud, cloud_filtered);
  cloud_filtered.header.frame_id = cloud->header.frame_id;

  pub.publish(cloud_filtered);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "crop_box");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/crop_box", 1);
  ros::spin();
}