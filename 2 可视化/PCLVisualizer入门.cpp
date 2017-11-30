#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
	//定义一个点云cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = 1000;
	cloud->height = 1;
	cloud->resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->at(i).x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->at(i).y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->at(i).z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//显示类
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	//设置窗口背景颜色，范围为0-1
	viewer.setBackgroundColor(0, 0, 0);

	//添加坐标轴
	viewer.addCoordinateSystem(1000);

	//往窗口添加点云,第二个参数为点云ID，添加多个点云时，必须写上ID
	viewer.addPointCloud(cloud, "cloud");

	//修改点云后，可以使用下面的函数更新点云
	//viewer.updatePointCloud(cloud, "cloud");

	//从窗口里删除点云，可以使用下面函数
	//viewer.removePointCloud("cloud");

	//添加点云后，通过点云ID来设置显示大小
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

	//重置相机，将点云显示到窗口
	viewer.resetCamera();

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}