#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/filters/passthrough.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/sample_consensus/method_types.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    cloudptr pcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudptr pcloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud, *pcloud1);
    sensor_msgs::PointCloud2 cloud_filtered;

    // Passthrough Filter

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pcloud1);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 0.3);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, 0.1);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.4);
    pass.filter(*pcloud_filtered);

    // Ransac segmentation

    cloudptr pcloud_ransac_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // perform ransac planar filtration to remove table top
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> segment1;
    // Optional
    segment1.setOptimizeCoefficients(true);
    // Mandatory
    segment1.setModelType(pcl::SACMODEL_PLANE);
    segment1.setMethodType(pcl::SAC_RANSAC);
    segment1.setDistanceThreshold(0.008);

    segment1.setInputCloud(pcloud_filtered);
    segment1.segment(*inliers, *coefficients);

    // Extract by filtering
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    //extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setInputCloud(pcloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pcloud_ransac_filtered);

    pcl::toROSMsg(*pcloud_ransac_filtered, cloud_filtered);
    cloud_filtered.header.frame_id = cloud->header.frame_id;

    pub.publish(cloud_filtered);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_pcl_ros_seg");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/test_seg", 1);
    ros::spin();
}