#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_type.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h> //represents the Nodelet segmentation class for Sample Consensus methods and models, it's a wrapper for generic-purpose SAC_based segmentation

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

struct downsampleParameters
{
    float leafsize;
    float limit_min;
    float limit_max;
};

struct normalEstimationParameters
{

    float radius_;
    float ksearch_;
    int thread_nums; // for omp
};

class PlanarFinder
{
    using PointT = pcl::PointXYZ;
    using PointNT = pcl::Normal;

public:
    explicit PlanarFinder();
    ~PlanarFinder();

    /**
     * @brief downsample the input point cloud to accelerate subsequent algorithms
     * 
     * @param cloudin input data
     * @param params user defined algorithm parameters
     * @return pcl::PointCloud<PointT>::Ptr A pointer to point cloud with pcl::PointXYZ
     */
    static pcl::PointCloud<PointT>::Ptr
    downsample(const pcl::PointCloud<PointT>::Ptr &cloudin, downsampleParameters params);

    /**
     * @brief compute specified surface normal 
     * 
     * @param cloudin input cloud data 
     * @param params user defined algorithm parameters
     * @return pcl::PointCloud<PointNT>::Ptr A pointer to point cloud with pcl::PointXYZ
     */
    static pcl::PointCloud<PointNT>::Ptr
    surfaceNormalEstimation(const pcl::PointCloud<PointNT>::Ptr &cloudin, normalEstimationParameters params);

    /**
     * @brief compute planar equation's coefficients
     * 
     * @param cloudin input cloud data 
     * @param params user defined algorithm parameters
     * @return pcl::ModelCoefficientsPtr A pointer to class ModelCoefficients
     */
    static pcl::ModelCoefficientsPtr
    fitPlane(const pcl::PointCloud<PointT>::Ptr &cloudin, ParametersForFitting params);

    /**
     * @brief compute planar equation's coefficients
     * 
     * @param cloudin input cloud data 
     * @param params user defined algorithm parameters
     * @return pcl::PointCloud<PointT>::Ptr A pointer to point cloud with pcl::PointXYZ
     */
    static pcl::PointCloud<PointT>::Ptr
    extractPlane(const pcl::PointCloud<PointT>::Ptr &cloudin, ParametersForFitting params);

    /**
     * @brief Use EuclidieanClusterExtraction to group a cloud into contiguous clusters
     * 
     * @param cloudin input cloud data
     * @param params user defined algorithm parameters
     */
    void clusterPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudin,
                       clusterSegParameters params, std::vector<pcl::PointIndices> &cluster_indices_out);
};