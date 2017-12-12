#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>   //����������Ʒ���ͷ�ļ�
#include <pcl/sample_consensus/model_types.h>   //ģ�Ͷ���ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ�����ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
	// �������ݣ���������������Ƶ�x,y���꣬������zΪ1��ƽ����
	cloud->width = 1000;cloud->height = 1;cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f);
		if(i%20==0)		cloud->points[i].z = 0.7+0.6*rand() / (RAND_MAX + 1.0);//���������Ⱥ��
		else			cloud->points[i].z = 1.0;
	}

	//�����ָ�ʱ����Ҫ��ģ��ϵ������coefficients�����洢�ڵ�ĵ��������϶���inliers��
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ�����ã�����ģ��ϵ����Ҫ�Ż�
	seg.setOptimizeCoefficients(true);
	// ��Ҫ�����ã����÷ָ��ģ�����ͣ����õ�����������Ʒ��������뷧ֵ���������
	seg.setModelType(pcl::SACMODEL_PLANE);   //����ģ������-ƽ��ģ��ax+by+cz+d=0
	seg.setMethodType(pcl::SAC_RANSAC);      //�����������һ���Է�������
	seg.setDistanceThreshold(0.1);    //�趨���뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ��Ǳ������������
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	//����ɸ�ָ��ʣ�µĵ㵽cloudout
	pcl::copyPointCloud(*cloud, *inliers, *cloudout);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	//��ӡ��ƽ��ģ��coefficients[4]
	std::cout << "  xa=" << coefficients->values[0]
			  << "  yb=" << coefficients->values[1]
			  << "  zc=" << coefficients->values[2]
			  << "  d =" << coefficients->values[3] << endl;
	std::cout <<"totoal number:"<< cloudout->points.size()<< endl;
	 //X��ƽ�Ʊ��ڹ۲�
	for (size_t i = 0; i < cloudout->points.size(); ++i)
	{
		cloudout->points[i].x = cloudout->points[i].x-1.0;
	}
	//���ӻ�,�ֱ��ϲ�ͬ��ɫ
	pcl::visualization::PCLVisualizer viewer("main window");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, green_color, "cloud1");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(cloudout, 0,0,255);
	viewer.addPointCloud<pcl::PointXYZ>(cloudout, blue_color, "cloud2");

	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters(); 
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return (0);
}