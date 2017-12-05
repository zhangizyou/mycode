#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
	//����һ���洢���Ƶ�PointCloud���ʵ����ʹ��PointXYZ�ṹʵ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	//���õ��ƵĴ�С
	cloud->width = 20000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	//������õ�����������
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = rand() / (RAND_MAX + 1.0f);
	}
	//����ɸѡ�㣬�������������㵽cloudOut��
	std::vector<int> indexs;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		float px, py, pz;
		px = cloud->points[i].x;
		py = cloud->points[i].y;
		pz = cloud->points[i].z;
		if (px<0.1 || py<0.1 || pz<0.1) indexs.push_back(i); //ɸѡ�߽��
	}
	//������������
	pcl::copyPointCloud(*cloud, indexs, *cloudOut);//in --index---out
	
	//��ɫ�����ʾ
	pcl::visualization::PCLVisualizer viewer("mainwindow");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_cloud_color_handler(cloud, 0, 200, 0);
	viewer.addPointCloud(cloud, model_cloud_color_handler, "name1");//ɸѡǰ

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloudOut, 200, 200, 200);
	viewer.addPointCloud(cloudOut, source_cloud_color_handler, "name2");//ɸѡ��

	viewer.addCoordinateSystem(1);
	viewer.initCameraParameters();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;

}