#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/impl/io.hpp>

char filepath1[] = "E:\\PCL\\PCDFILE\\pcl_logo.pcd";

int main() {
	//数据获取
	pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filepath1, *cloud) != 0) return -1;

	//数据处理--修改点云颜色
	//设定修改范围位前后10%的数据
	size_t parameter1, parameter2;
	parameter1 = cloud->points.size() * 1 / 10;
	parameter2 = cloud->points.size() * 9 / 10;
	//方法1：遍历
	for (size_t i = 0; i < parameter1; i++)
	{
		cloud->at(i).rgba = 0xff0000;  //red
	}
	//方法2：迭代
	pcl::PointCloud<pcl::PointXYZRGBA>::iterator it, end;
	for (it = cloud->begin() + parameter2, end = cloud->end(); it != end; ++it)
	{
		(*it).rgba = 0x0000ff;//blue
	}

	//数据显示(两种方法)
	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "3D Viewer");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	*/
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "3D Viewer");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}
