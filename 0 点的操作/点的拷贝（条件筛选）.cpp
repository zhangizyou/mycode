#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
	//定义一个存储点云的PointCloud类的实例，使用PointXYZ结构实例化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);

	//设置点云的大小
	cloud->width = 20000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	//随机设置点云里点的坐标
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = rand() / (RAND_MAX + 1.0f);
	}
	//条件筛选点，并按索引拷贝点到cloudOut中
	std::vector<int> indexs;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		float px, py, pz;
		px = cloud->points[i].x;
		py = cloud->points[i].y;
		pz = cloud->points[i].z;
		if (px<0.02 || py<0.02 || pz<0.02) indexs.push_back(i); //筛选边界点
	}

	pcl::copyPointCloud(*cloud, indexs, *cloudOut);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloudOut, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addCoordinateSystem(1.0);//添加坐标轴
	viewer->initCameraParameters();  //添加视角
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	return 0;
}