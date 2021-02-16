#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    cout<<"callback"<<endl;
    cloudptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudptr point_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud, *point_cloud);
    sensor_msgs::PointCloud2 cloud_filtered;

    // NARF filter

    float angularResX = (float)(64.0f / 640.0f * (M_PI / 180.0f));
    float angularResy = (float)(41.0f / 480.0f * (M_PI / 180.0f));

    float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
    float maxAngleY = (float)(50.0f * (M_PI / 180.0f));

    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(point_cloud->sensor_origin_[0], point_cloud->sensor_origin_[1], point_cloud->sensor_origin_[2])) * Eigen::Affine3f(point_cloud->sensor_orientation_);

    float noiseLevel = 0.0f;
    float minimumRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeimage;
    rangeimage.createFromPointCloud(*point_cloud, angularResX, angularResy, maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel, minimumRange, borderSize);
    // rangeimage.
    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(point_cloud);
    // sor.setLeafSize(0.05, 0.05, 0.05);
    // sor.filter(*point_cloud_filtered);

    // Visualize the image.
    cout<<"BEFORE"<<endl;
    pcl::visualization::RangeImageVisualizer viewer("Planar range image");
    viewer.showRangeImage(rangeimage);
    // viewer.vi
    // while (!viewer.wasStopped())
    // {
    //     viewer.spinOnce();
    //     cout<<"inside"<<endl;
    //     // Sleep 100ms to go easy on the CPU.
    //     pcl_sleep(0.1);
        
    // }
    // pcl::toROSMsg(*, cloud_filtered);
    // cloud_filtered.header.frame_id = cloud->header.frame_id;

    pub.publish(cloud_filtered);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_pcl_ros");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/test", 1);
    cout<<"msin"<<endl;
    ros::spin();
}