#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
	//����һ������cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = 1000;
	cloud->height = 1;
	cloud->resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->at(i).x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->at(i).y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->at(i).z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//��ʾ��
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	//���ô��ڱ�����ɫ����ΧΪ0-1
	viewer.setBackgroundColor(0, 0, 0);

	//���������
	viewer.addCoordinateSystem(1000);

	//��������ӵ���,�ڶ�������Ϊ����ID����Ӷ������ʱ������д��ID
	viewer.addPointCloud(cloud, "cloud");

	//�޸ĵ��ƺ󣬿���ʹ������ĺ������µ���
	//viewer.updatePointCloud(cloud, "cloud");

	//�Ӵ�����ɾ�����ƣ�����ʹ�����溯��
	//viewer.removePointCloud("cloud");

	//��ӵ��ƺ�ͨ������ID��������ʾ��С
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

	//�����������������ʾ������
	viewer.resetCamera();

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}