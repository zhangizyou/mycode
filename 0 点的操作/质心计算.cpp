#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

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
		if (px<0.02 || py<0.02) indexs.push_back(i); //ɸѡ�߽��
	}

	pcl::copyPointCloud(*cloud, indexs, *cloudOut);

	//�����洢�������ĵĶ���
	Eigen::Vector4f centroid1, centroid2;
	pcl::compute3DCentroid(*cloud, centroid1);	//��������
	pcl::compute3DCentroid(*cloudOut, centroid2);	//��������
	std::cout << "��������Ϊ("
		<< centroid1[0] << ", "
		<< centroid1[1] << ", "
		<< centroid1[2] << ")." << std::endl;

	std::cout << "ɸѡ��ĵ�������Ϊ("
		<< centroid2[0] << ", "
		<< centroid2[1] << ", "
		<< centroid2[2] << ")." << std::endl;

	return 0;
}