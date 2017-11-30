#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/impl/io.hpp>

char filepath1[] = "B:\\PCL\\PCDFILE\\pcl_logo.pcd";

int main() {
	//���ݻ�ȡ
	pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filepath1, *cloud) != 0) return -1;

	//���ݴ���--�޸ĵ�����ɫ
	//����1������
	for (size_t i = 0; i < 2500; i++)
	{
		cloud->at(i).rgba = 0xff0000;  //RGB-��ɫ0XFF0000
	}
	//����2������
	pcl::PointCloud<pcl::PointXYZRGBA>::iterator it, end;
	for (it = cloud->begin() + 10000, end = cloud->end(); it != end; ++it)
	{
		(*it).rgba = 0x0000ff;
	}

	//������ʾ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return 0;
}
