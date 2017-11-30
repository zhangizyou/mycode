#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

int
main(int argc, char** argv)
{
	srand(time(NULL));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
	//Generate KD������
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	//�趨һ���ο���
	pcl::PointXYZ searchPoint(200,200,200); 

	// K nearest neighbor search
	std::cout << "����(200,200,200)�����10���㣺" <<std::endl;
	int K = 10;							          //�����10����
	std::vector<int> pointIdxNKNSearch(K);        //�����Ŵ��
	std::vector<float> pointNKNSquaredDistance(K);//��Ӧ�ľ���
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) //if search some point
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// Neighbors within radius search----------------
	std::cout << "��(200,200,200)�뾶��Χ120���ڵĵ��У�" << std::endl;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 120; //set the search radius
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}

	return 0;
}