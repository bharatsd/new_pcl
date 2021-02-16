#ifndef PCL_UTILITIES_H
#define PCL_UTILITIES_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes

#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

namespace PCLUtil
{
    template <typename PointT>
    pcl::PointCloud<PointT> voxel_downsampling(pcl::PointCloud<PointT> input_cloud, vector<double> leafXYZ)
    {
        typename pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>);
        typename pcl::VoxelGrid<PointT> voxelObj;
        voxelObj.setInputCloud(input_cloud.makeShared());
        voxelObj.setLeafSize(leafXYZ[0], leafXYZ[1], leafXYZ[2]);
        voxelObj.filter(*downsampled_cloud);

        return *downsampled_cloud;
    }

    void publishonRviz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, ros::Publisher pub, std::string frame_id)
    {
        sensor_msgs::PointCloud2 cloud_to_pub;
        pcl::toROSMsg(*input_cloud, cloud_to_pub);
        cloud_to_pub.header.frame_id = frame_id;
        pub.publish(cloud_to_pub);
    }
} // namespace PCLUtil

#endif