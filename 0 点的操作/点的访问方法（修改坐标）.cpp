#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/impl/io.hpp>

char filepath1[] = "E:\\PCL\\PCDFILE\\pcl_logo.pcd";

int main() {
	//���ݻ�ȡ
	pcl::PointCloud<pcl::PointXYZRGBA> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filepath1, *cloud) != 0) return -1;

	//���ݴ���--�޸ĵ�����ɫ
	size_t parameter1;
	parameter1 = cloud->points.size() ;
	//�޸�Z������
	for (size_t i = 0; i < parameter1; i++)
	{
		cloud->at(i).z = i / 10000.0;//i%5;  
	}

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "3D Viewer");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
	viewer.addCoordinateSystem(1);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;
}
