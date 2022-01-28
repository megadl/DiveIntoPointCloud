// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
// std
#include <iostream>
#include <thread>

using namespace std::chrono_literals;
/**
 * @brief multiViewPort viewer
 * 
 * @tparam T should be point types supported by <point_types.h>, eg., pcl::PointXYZ
 * @param cloud reference to the input cloud
 * @param cloud_f reference to the output cloud
 * @return pcl::visualization::PCLVisualizer::Ptr 
 */
template <typename T>
pcl::visualization::PCLVisualizer::Ptr multiViewer(pcl::PointCloud<T>::ConstPtr &cloud, pcl::PointCloud<T>::ConstPtr &cloud_f)
{
    pcl::visualization::PCLVisualizer::Ptr viewer{"multiviewer"};

    int v1{0}, v2{0};
    viewer->setBackgroundColor(0,0,0,v1);
    viewer->setBackgroundColor(0,255,255,v2);

    viewer->addPointCloud(cloud, "input cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input cloud");
    viewer->initCameraParameters();

    viewer->addPointCloud(cloud_f, "filtered cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered cloud");
    viewer->initCameraParameters();

    return viewer;
}

template <typename T>
void ransacPipeline(pcl::PointCloud<T>::Ptr &cloud, pcl::PointCloud<T>::Ptr &cloud_f)
{
    // load pcd file, pointcloud
    if(pcl::io::loadPCDFile("D:/pcd/milk.pcd", *cloud) < 0)
        PCL_ERROR("Error loading target pcd file...");
    
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.15f,0.15f,0.15f);
    sor.filter(*cloud_f);

    // ransac
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
}