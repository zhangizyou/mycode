#include <iostream>           //��׼C++���е�������������ͷ�ļ���
#include <pcl/io/pcd_io.h>   //pcd ��д����ص�ͷ�ļ���
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include <pcl/visualization/cloud_viewer.h> //������OpenCV��highgui
#include <pcl/common/impl/io.hpp>

//��������Դ--------------------------------------------------------------
char filepath[] = "E:\\PCL\\PCDFILE\\pcl_logo.pcd";
//������--------------------------------------------------------------------------
int
main()
{
	//���岢��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1)//�򿪵����ļ�
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
	//ɾ����
	pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
	cloud->erase(index + 2000, index + 4000); //ɾ��һ������ĵ�

	//��ӵ�
	pcl::PointXYZ p1;
	p1.x = 0.5;
	p1.y = 0.5;
	p1.z=cloud->at(0).z-0.5;
	cloud->insert(index+300,p1);A
	//cloud->erase(index); //ɾ��һ����	
	//push_back(point)//׷��һ����
	//insert(index, point); //��indexλ�ò����
	//insert(index, N, point); //��indexλ�ò���N����


	//�����Ӵ���CloudViewer�Ǽ���ʾ���ƵĿ��ӻ������࣬�����Ӵ��಻��Ӧ���ڶ��߳�Ӧ�ó����С�
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped());

	return (0);
}




