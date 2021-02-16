#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <my_pcl_tutorial/pcl_utilities.h>
#include <vector>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  cloudptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloudptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *point_cloud);
  sensor_msgs::PointCloud2 cloud_filtered;
  vector<double> leaf = {0.01f,0.01f,0.01f};
  std::string f_id = cloud->header.frame_id;

  // Voxel Grid Filter 

  *filtered_cloud = PCLUtil::voxel_downsampling(*point_cloud,leaf);
  PCLUtil::publishonRviz(filtered_cloud,pub,f_id);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "voxel_test");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/test", 1);
  ros::spin();
}