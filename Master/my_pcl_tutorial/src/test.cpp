#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace std;
ros::Publisher pub;
// typedef pcl::PointXYZRGB rgbpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
typedef cloudrgb::Ptr cloudrgbptr;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  cloudrgbptr PC (new cloudrgb());
  cloudrgbptr PC_filtered (new cloudrgb());
  pcl::fromROSMsg(*cloud, *PC); //Now you can process this PC using the pcl functions 
  sensor_msgs::PointCloud2 cloud_filtered;

   pcl::VoxelGrid<pcl::PointXYZRGB> sor ;
   sor.setInputCloud (PC);
   sor.setLeafSize (0.01, 0.01, 0.01);
   sor.filter (*PC_filtered);

  pcl::toROSMsg(*PC_filtered, cloud_filtered);
  cloud_filtered.header.frame_id = cloud->header.frame_id;
  
  pub.publish (cloud_filtered);
}
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_pcl_ros");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/color/image_raw", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/rgb/test", 1);
    ros::spin();
}