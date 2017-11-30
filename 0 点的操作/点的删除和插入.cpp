#include <iostream>           //标准C++库中的输入输出类相关头文件。
#include <pcl/io/pcd_io.h>   //pcd 读写类相关的头文件。
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/visualization/cloud_viewer.h> //类似于OpenCV的highgui
#include <pcl/common/impl/io.hpp>

//定义点云---------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
char filepath[] = "B:\\PCL\\PCDFILE\\pcl_logo.pcd";
//主函数--------------------------------------------------------------------------
int
main()
{
	//读取
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1)//打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
	//添加删除点
	pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
	//cloud->erase(index); //删除一个点	
	cloud->erase(index+2000,index+4000); //删除一个区间的点

	//push_back(point)//追加点
	//insert(index, point); //在index位置插入点
	//insert(index, N, point); //在index位置插入N个点

	//点云视窗类CloudViewer是简单显示点云的可视化工具类，点云视窗类不能应用于多线程应用程序中。
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped());
	
	return (0);
}




