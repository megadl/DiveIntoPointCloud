#include "find_planar.h"



pcl::PointCloud<PointT>::Ptr
PlanarFinder::downsample(const pcl::PointCloud<PointT>::Ptr &cloudin, downsampleParameters params)
{
    std::cerr << "Now [downsample] starts %s", tt.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f; // downsampled cloud --> output

    if(!params.centroid)
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> so;
    else
        pcl::VoxelGrid<pcl::PointXYZ> so;

    so.setInputCloud(cloudin);
    so.setFilterLimits(params.limit_min, params.limit_max);
    so.setLeafSize(params.leafsize, params.leafsize, params.leafsize);
    so.filter(*cloud_f);
    std::cerr << "Now [downsample] ends %s", tt.toc();

    return cloud_f;
}


pcl::PointCloud<PointNT>::Ptr
PlanarFinder::surfaceNormalEstimation(const pcl::PointCloud<PointT>::Ptr &cloudin, normalEstimationParameters params)
{
    std::cerr << "Now [downsample] starts %s", tt.tic();
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::search::KdTree<PointT>::Ptr tree;
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setInputCloud(cloudin);
    ne.setSearchMethod(tree)
    ne.setRadiusSearch(params.radius_);
    ne.compute(*normals);
    std::cerr << "Now [downsample] ends %s", tt.toc();
    
    return normals;
}

 
pcl::ModelCoefficients::Ptr
PlanarFinder::fitPlane (const PointCloudPtr & input, float distance_threshold, float max_iterations)
{
    // Initialize the SACSegmentation object
    std::cerr << "Now [fitPlane] starts, %s", tt.tic();
    pcl::SACSegmentation<PointT> seg;
    // optional
    seg.setOptimizeCoefficients (true);
    // mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);
    seg.setMaxIterations (max_iterations);

    seg.setInputCloud (input);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    seg.segment (*inliers, *coefficients);  

    std::cerr << "Now [fitPlane] ends, %s", tt.toc();
    return (coefficients);
}

pcl::PointCloud<PointT>::Ptr
PlanarFinder::extractPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudin, ParametersForFitting params)
{
    pcl::ModelCoefficients::Ptr coe(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());// PointIndices is wrapper for pcl::Indices
    // segment and the extract inliers and outliers
    pcl::SACSegmentation<PointT> seg;
    seg.setInputCloud(cloudin);
    seg.setMaxIterations(params.max_iterations); // maybe 100
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params.distance_threshold); // maybe 0.02

    seg.segment(*inliers, *coe);

    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_remained(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudin);
    extract.setIndices(inliers);
    extract.setNegative(false); // filter output is inliers
    extract.filter(*cloud_f); // cloud_f should be the target plane

    extract.setNegative(true); // filter output is outliers
    extract.filter(*cloud_remained);

    // use KdTree for search method of the extraction => clustering the remaining to chuncks
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_remained);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_remained);
    ec.extract(clusters);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : it->indices)
        cloud_cluster->emplace_back((*cloud_filtered)[idx]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
}
