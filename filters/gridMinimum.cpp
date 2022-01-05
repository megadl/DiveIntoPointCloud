//GridMinimum assembles a local 2D grid over a given PointCloud, and downsamples the data with the minimum z value
//GridMinimum can be useful in a number of topographic processing tasks such as crudely estimating ground returns, especially under foliage.
// �ڵ��δ��������бȽ����ã�����ֱ��ɾ�����淵����Ϣ���ر�������Ҷ�µ����������Ҳ���Ǹ߳��жϵ��������Ҷĩ�˺͵���֮��û�и̣߳���
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
	pcl::GridMinimum<pcl::PointXYZ> sor(resolution); // ������voxelgrid��approximate_voxelgrid�����ݹ��캯����Ҫ����2D����߳�,û��default constructor
	//sor.setResolution(resolution);
	sor.setInputCloud(cloud_shared_ptr);
	//sor.filter(cloud_shared_ptr_f);
	sor.applyFilter(cloud_shared_ptr_f);
}