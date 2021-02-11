#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  cloudptr PC(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloudptr PC_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *PC);
  sensor_msgs::PointCloud2 cloud_filtered;

  // Voxel Grid Filter 

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(PC);
  sor.setLeafSize(0.05, 0.05, 0.05);
  sor.filter(*PC_filtered);

  pcl::toROSMsg(*PC_filtered, cloud_filtered);
  cloud_filtered.header.frame_id = cloud->header.frame_id;

  pub.publish(cloud_filtered);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pcl_ros_voxel_orig");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/test", 1);
  ros::spin();
}