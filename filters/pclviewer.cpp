#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

enum class FilterType {
    PassThrough,
    VoxelGrid,
    StatisticalOutlierRemoval,
    RadiusOutlierRemoval,
    ConditionalRemoval,
    ExtractIndices,
    ProjectInliers,
    CropBox,
    CropHull
};

pcl::PointCloud<pcl::PointXYZ> filterPipeline(int filtertype){
            pcl::PassThrough<pcl::PointXYZ> pass;
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor3;
            pcl::ConditionalRemoval<pcl::PointXYZ> sor4;
            pcl::ExtractIndices<pcl::PointXYZ> sor5;
            pcl::ProjectInliers<pcl::PointXYZ> sor6;
            pcl::CropBox<pcl::PointXYZ> sor7;
            pcl::CropHull<pcl::PointXYZ> sor8;
    switch (filtertype) {
        case 0:
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.0, 1.0);
            pass.filter(*cloud);
            break;
        case 1:
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f, 0.01f, 0.01f);
            sor.filter(*cloud);
            break;
        case 2:
            sor2.setInputCloud(cloud);
            sor2.setMeanK(50);
            sor2.setStddevMulThresh(1.0);
            sor2.filter(*cloud);
            break;
        case 3:
            sor3.setInputCloud(cloud);
            sor3.setRadiusSearch(0.05);
            sor3.setMinNeighborsInRadius(50);
            sor3.filter(*cloud);
            break;
        case 4:


            break;
        case 5:
            sor5.setInputCloud(cloud);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            for (int i = 0; i < cloud->points.size(); i++) {
                if (cloud->points[i].z < 0.5) {
                    inliers->indices.push_back(i);

                }
            break;
    
    return *cloud;
}

void main(int argc, char* argv[])
{
    FilterType filtertype = FilterType::PassThrough;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");


    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud(cloud, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}