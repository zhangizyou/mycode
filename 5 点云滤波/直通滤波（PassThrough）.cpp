#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>   //pcd ��д����ص�ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
//=====================================================================����ԭʼ����λ��
char filepath[] = "B:\\PCL\\PCDFILE\\rabbit.pcd";
//======================================================================main()
int main(int argc, char** argv)
{
	//-----------------------------���岢��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);

	//----------------------------�˲����������ƣ�ֱͨ�˲�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> Myfilter;
	Myfilter.setInputCloud(cloud);            //�����������
	Myfilter.setFilterFieldName("y");         //���ù�����
	Myfilter.setFilterLimits(0.0, 2.0);        //�����ڹ����ֶεķ�Χ
	Myfilter.setFilterLimitsNegative (true);   //���ñ�����Χ�ڻ��ǹ��˵���Χ�ڣ�true����
	Myfilter.filter(*cloudOut);            //ִ���˲���������˽��

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