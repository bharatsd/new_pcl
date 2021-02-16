#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <my_pcl_tutorial/pcl_utilities.h>


#include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    cloudptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudptr point_cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud, *point_cloud_voxel);
    sensor_msgs::PointCloud2 cloud_filtered;
    vector<double> leaf = {0.005f,0.005f,0.005f};

    *point_cloud = PCLUtil::voxel_downsampling(*point_cloud_voxel,leaf);


    pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter(true);
    cropBoxFilter.setInputCloud(point_cloud);
    Eigen::Vector4f min_pt(-0.2f, -0.1f, -1.0f, 1.0f);
    Eigen::Vector4f max_pt(0.3f, 0.3f, 1.0f, 1.0f);

    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);

    cropBoxFilter.filter(*filtered_point_cloud);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100000);
    seg.setDistanceThreshold(0.005);
    int i = 0, nr_points = (int)filtered_point_cloud->size();
    while (filtered_point_cloud->size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(filtered_point_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(filtered_point_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *filtered_point_cloud = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(filtered_point_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01); // 2cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(4000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_point_cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*filtered_point_cloud)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.emplace_back(cloud_cluster);
    }

    cloudptr point_cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);
    point_cloud_final = clusters[0];

    pcl::toROSMsg(*point_cloud_final, cloud_filtered);
    cloud_filtered.header.frame_id = cloud->header.frame_id;

    pub.publish(cloud_filtered);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "crop_box_and_eucl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/crop_box_and_eucl", 1);
    ros::spin();
}