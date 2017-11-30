#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>   //pcd ��д����ص�ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
//=====================================================================����ԭʼ����λ��
char filepath[] = "B:\\PCL\\PCDFILE\\rabbit.pcd";
//======================================================================main()
int main(int argc, char** argv)
{
	//-----------------------------���岢��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);	//���ô��ڱ�����ɫ����ΧΪ0-1
	viewer.addCoordinateSystem(10);	//���������

	//���ֱ��
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(10, 10, 0), pcl::PointXYZ(10, 10, 10), "line");
	//�������
	viewer.addSphere(pcl::PointXYZ(20, 20, 20), 5, 0, 1, 0, "sphere");
	//���������
	viewer.addCube(0, 5, 0, 5, 0, 5, 1, 0, 0, "cube");
	//ɾ����Щ��״ʹ�����溯��,������״��Ӧ��ID
	//viewer.removeShape("cube");

	viewer.addPointCloud(cloud, "cloud1");//������cloud1���ԭʼ����,
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	//-----------------------------------�����������������ʾ������
	viewer.resetCamera();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}