#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>   //pcd 读写类相关的头文件。
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
//=====================================================================定义原始数据位置
char filepath[] = "B:\\PCL\\PCDFILE\\rabbit.pcd";
//======================================================================main()
int main(int argc, char** argv)
{
	//-----------------------------定义并读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud);

	//----------------------------滤波处理结果点云（离群点）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Myfilter;   //创建滤波器对象
	Myfilter.setInputCloud(cloud);                           //设置待滤波的点云
	Myfilter.setMeanK(20);                               //设置在进行统计时考虑查询点临近点数
	Myfilter.setStddevMulThresh(2.0);                      //设置判断是否为离群点的阀值
	Myfilter.filter(*cloudOut);                    //存储

	//-----------------------------显示对比,（结果点云着绿色）
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0, 0, 0);	//设置窗口背景颜色，范围为0-1
	viewer.addCoordinateSystem(10);	//添加坐标轴

	viewer.addPointCloud(cloud, "cloud1");//往窗口cloud1添加原始点云,
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Mycolor(cloudOut, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloudOut, Mycolor, "cloud2"); //着色
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");//往窗口cloud2添加结果点云,
	//-----------------------------------重置相机，将点云显示到窗口
	viewer.resetCamera();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}