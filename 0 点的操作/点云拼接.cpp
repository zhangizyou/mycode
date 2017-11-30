#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
	//����һ������cloudA
	pcl::PointCloud<pcl::PointXYZ> cloudA;
	cloudA.width = 3;
	cloudA.height = 1;
	cloudA.points.resize(cloudA.width * cloudA.height);
	for (size_t i = 0; i < cloudA.points.size(); ++i)
	{
		cloudA.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudA.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudA.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloudA.points.size(); ++i)
		std::cerr << "    "
		<< cloudA.points[i].x << " "
		<< cloudA.points[i].y << " "
		<< cloudA.points[i].z << std::endl;

	//����һ������cloudB
	pcl::PointCloud<pcl::PointXYZ> cloudB;
	cloudB.width = 5;
	cloudB.height = 1;
	cloudB.points.resize(cloudB.width * cloudB.height);
	for (size_t i = 0; i < cloudB.points.size(); ++i)
	{
		cloudB.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudB.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudB.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	std::cerr << "Cloud B: " << std::endl;
	for (size_t i = 0; i < cloudB.points.size(); ++i)
		std::cerr << "    "
		<< cloudB.points[i].x << " "
		<< cloudB.points[i].y << " "
		<< cloudB.points[i].z << std::endl;

	//C=A+B
	pcl::PointCloud<pcl::PointXYZ> cloudC = cloudA;
	cloudC += cloudB;
	std::cerr << "Cloud C=A+B: " << std::endl;
	for (size_t i = 0; i < cloudC.points.size(); ++i)
		std::cerr << "    "
		<< cloudC.points[i].x << " "
		<< cloudC.points[i].y << " "
		<< cloudC.points[i].z << " " << std::endl;


	//����һ���洢���ߵĵ���
	pcl::PointCloud<pcl::Normal> normalB;
	normalB.width = 5;
	normalB.height = 1;
	normalB.points.resize(normalB.width * normalB.height);
	for (size_t i = 0; i < normalB.points.size(); ++i)
	{
		normalB.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
		normalB.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
		normalB.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "normal B: " << std::endl;
	for (size_t i = 0; i < normalB.points.size(); ++i)
		std::cerr << "    "
		<< normalB.points[i].normal[0] << " "
		<< normalB.points[i].normal[1] << " "
		<< normalB.points[i].normal[2] << std::endl;

	//����һ�����Դ洢���ߺ�����ĵ���,����cloudB��������normalB��ķ��ߺϲ���cloudD
	pcl::PointCloud<pcl::PointNormal> cloudD;
	pcl::concatenateFields(cloudB, normalB, cloudD); //PointXYZ+Normal->PointNormal
	std::cerr << "Cloud D=B-(cat)-normalB: " << std::endl;
	for (size_t i = 0; i < cloudD.points.size(); ++i)
		std::cerr << "    "
		<< cloudD.points[i].x << " "
		<< cloudD.points[i].y << " "
		<< cloudD.points[i].z << " "
		<< cloudD.points[i].normal[0] << " "
		<< cloudD.points[i].normal[1] << " "
		<< cloudD.points[i].normal[2] << std::endl;

	return (0);
}