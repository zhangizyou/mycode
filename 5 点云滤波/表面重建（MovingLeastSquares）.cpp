#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>   //pcd ��д����ص�ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <boost/thread/thread.hpp>
//=====================================================================����ԭʼ����λ��
char filepath[] = "B:\\PCL\\PCDFILE\\rabbit.pcd";
//======================================================================main()
int main(int argc, char** argv)
{
	//-----------------------------���岢��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);

	//----------------------------����������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> Myfilter;
	Myfilter.setInputCloud(cloud);
	Myfilter.setSearchRadius(0.3);	// Use all neighbors in a radius of 30cm.
	// If true, the surface and normal are approximated using a polynomial estimation
	// (if false, only a tangent one).
	Myfilter.setPolynomialFit(true);
	Myfilter.setComputeNormals(true);// We can tell the algorithm to also compute smoothed normals (optional).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;// kd-tree object for performing searches.
	Myfilter.setSearchMethod(kdtree);
	Myfilter.process(*cloudOut);

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