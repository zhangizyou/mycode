//����KdTree��������ܶȵ�һ�ַ���----���������ڵ��ƽ������ʾ��ԽС�ܶ�Խ��
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

char filepath[] = "E:\\PCL\\PCDFILE\\table_scene_lms400.pcd"; //pcl_logo.pcd//table_scene_lms400.pcd

float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k)
{
	double res = 0.0; //�����
	int n_points = 0; //��ĸ���

	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		pcl::PointXYZ Refp = (*cloud)[i]; //ÿһ�����صĲο���
		if (!pcl_isfinite(Refp.x))  continue;//������Ч����
		std::vector<int> indices(k);
		std::vector<float> sqr_distances(k);
		if (tree.nearestKSearch(Refp, k, indices, sqr_distances) == k) //����ҵ����㹻��ĵ�
		{
			for (int i = 1; i < k; i++)
			{
				res += sqrt(sqr_distances[i]);
				++n_points;
			}
		}
	}
	if (n_points != 0)  res /= n_points;
	return res;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(filepath, *cloud);//���ļ���һ��Ŀ¼�µ�3d_pcd\\rabbit.pcd

	float des = computeCloudResolution(cloud, 4);
	std::cout << des;

	return 0;
}