#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>   //pcd ��д����ص�ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
//=====================================================================����ԭʼ����λ��
char filepath[] = "B:\\PCL\\PCDFILE\\rabbit.pcd";
//======================================================================main()
int main(int argc, char** argv)
{
	//-----------------------------���岢��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);

	//----------------------------�˲����������ƣ�ͶӰ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	//����һ��ƽ��
	pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients());
	plane->values.resize(4);
	plane->values[0] = 0;
	plane->values[1] = 0;
	plane->values[2] = 1.0;
	plane->values[3] = 0;
	pcl::ProjectInliers<pcl::PointXYZ> project;
	project.setInputCloud(cloud);
	project.setModelType(pcl::SACMODEL_PLANE);//����ͶӰģ��
	project.setModelCoefficients(plane);//����ͶӰģ�Ͳ���
	project.filter(*cloudOut);

	//-----------------------------��ʾ�Ա�,�������������ɫ��
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);	//���ô��ڱ�����ɫ����ΧΪ0-1
	viewer.addCoordinateSystem(10);	//���������

	viewer.addPointCloud(cloud, "cloud1");//������cloud1���ԭʼ����,
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Mycolor(cloudOut, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloudOut, Mycolor, "cloud2"); //��ɫ
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");//������cloud2��ӽ������,
	//-----------------------------------�����������������ʾ������
	viewer.resetCamera();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}