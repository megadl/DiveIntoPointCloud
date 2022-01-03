
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

void
ConvexHullFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected)
{

	pcl::PCDReader reader;
	reader.read("D:/pcd/table_scene_mug_stereo_textured.pcd", *cloud);
	//------------------------------------------------------------
	// Build a filter to remove spurious NaNs and scene background
	// 构建一个滤波器用来移除假的NaN和背景
	// -----------------------------------------------------------
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.1);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;
	//------------------------------------------------------------
	// use SACSegmentation to get inliers which fits SACMODEL_PLANE
	//------------------------------------------------------------
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

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_filtered);
	proj.setIndices(inliers);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	// Create a Convex Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_projected);
	chull.reconstruct(*cloud_hull);

	std::cerr << "Convex hull has: " << cloud_hull->size() << " data points." << std::endl;

	pcl::PCDWriter writer;
	writer.write("D:/pcd/table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

}


void _main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 存储输入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); // 存储过滤点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>); // 存储投影点云
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ConvexHullFilter"));
	ConvexHullFilter(cloud, cloud_filtered, cloud_projected);

	int v1{0}, v2{0}, v3{0};
	viewer->createViewPort(0, 0, 0.33, 1, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, red, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	// 设置显示属性
	//pcl::visualization::PointCloudColorHandlerCustom
	viewer->createViewPort(0.33, 0, 0.5, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_filtered, 0, 255, 0);
	viewer->addPointCloud(cloud_filtered, green, "filtered cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	viewer->createViewPort(0.5, 0, 1, 1, v3);
	viewer->setBackgroundColor(0, 0, 0, v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_projected, 0, 0, 255);
	viewer->addPointCloud(cloud_projected, blue, "projected cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "projected cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
}

