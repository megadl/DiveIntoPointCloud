//GridMinimum assembles a local 2D grid over a given PointCloud, and downsamples the data with the minimum z value
//GridMinimum can be useful in a number of topographic processing tasks such as crudely estimating ground returns, especially under foliage.
// 在地形处理任务中比较有用，可以直接删除地面返回信息，特别是在树叶下的那种情况，也就是高程中断的情况（树叶末端和地面之间没有高程）。
#include <pcl/filters/grid_minimum.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// std
#include <thread>

using namespace std::chrono_literals;
template <typename T1, typename T2>
void Grid_minimum(const T1& cloud_shared_ptr, T2& cloud_shared_ptr_f)
{
	constexpr float resolution = 0.1;
	pcl::GridMinimum<pcl::PointXYZ> sor(resolution); // 区别于voxelgrid和approximate_voxelgrid，依据构造函数需要给出2D网格边长,没有default constructor
	//sor.setResolution(resolution);
	sor.setInputCloud(cloud_shared_ptr);
	//sor.filter(cloud_shared_ptr_f);
	sor.applyFilter(cloud_shared_ptr_f);
}