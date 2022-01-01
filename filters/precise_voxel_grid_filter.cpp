#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <thread>

using namespace std::chrono_literals;

void preciseVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr sp)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>); // a new pointcloud
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(sp);
    voxel.setLeafSize(0.01f,0.01f,0.01f);
    voxel.filter(*cloud_f);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_f);
    pcl::PointIndicesPtr pIndices = std::shared_ptr<pcl::PointIndices>(new pcl::PointIndices());
    // 为降采样后的点云中的每个体素中心点都执行最近邻k=1搜索，并用最近邻点更新体素中心。
    // after voxel grid downsampling, do k-search
    for (auto& i = 0; i < voxel->points.size(); i++)
    {
        pcl::PointXYZ querypoint;
        querypoint.x = voxel->points[i].x;
        querypoint.y = voxel->points[i].y;
        querypoint.z = voxel->points[i].z;
        //--------------------------------
        // K nearest neighbor search
        //--------------------------------
        vector<int> pointIdxKNNSearch(K); // neighbor's indices
        vector<float> pointKNNSquaredDistance(K); // distance from neighbor to the querypoint
        // Search for k-nearest neighbors for the given query point. 
        int  K = 1;
        if(kdtree.nearestKsearch(querypoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
        {
            pIndices->indices.push_back(pointIdxKNNSearch);
        }
        //--------------------------------
        // Neighbors within radius R search
        //--------------------------------
        // vector<int> pointIdxRadiusSearch;
        // vector<float> pointRadiusSquaredDistance;
        // float radius = 1.0f;
        // if(kdtree.radiusSearch(querypoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        // {
        //     pIndices->indices.push_back(pointIdxRadiusSearch);
        // }
       
       
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f_fixed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_f, pIndices->indices, *cloud_f_fixed);

    pcl::PCDWriter writer;
    stringstream ss;
    ss<< "precise voxel grid downsample" << sp;
    pcl::io::savePCDFile(ss.str(), *cloud_f_fixed, true);
    
}
 