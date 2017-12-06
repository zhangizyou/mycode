#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


char filepath1[] = "E:\\PCL\\PCDFILE\\pcl_logo.pcd";

int main() {
	//���ݻ�ȡ
	pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filepath1, *cloud) != 0) return -1;

	//���ݴ���--�޸ĵ�����ɫ
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->at(i).rgba = rand()*0xffffff/(RAND_MAX+1.0);  //RGB-��ɫ0XFF0000
	}

	//��������
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