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

pcl::PointCloud<pcl::PointXYZ> filterPipeline(int filtertype);

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
