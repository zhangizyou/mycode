#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>   //pcd 读写类相关的头文件。
#include <pcl/visualization/pcl_visualizer.h>
//=====================================================================定义原始数据位置
char filepath[] = "B:\\PCL\\PCDFILE\\rabbit.pcd";
//======================================================================main()
int main(int argc, char** argv)
{
	//-----------------------------定义并读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);	//设置窗口背景颜色，范围为0-1
	viewer.addCoordinateSystem(10);	//添加坐标轴

	//添加直线
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(10, 10, 0), pcl::PointXYZ(10, 10, 10), "line");
	//添加球体
	viewer.addSphere(pcl::PointXYZ(20, 20, 20), 5, 0, 1, 0, "sphere");
	//添加正方形
	viewer.addCube(0, 5, 0, 5, 0, 5, 1, 0, 0, "cube");
	//删除这些形状使用下面函数,填上形状对应的ID
	//viewer.removeShape("cube");

	viewer.addPointCloud(cloud, "cloud1");//往窗口cloud1添加原始点云,
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	//-----------------------------------重置相机，将点云显示到窗口
	viewer.resetCamera();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}