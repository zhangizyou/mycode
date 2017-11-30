//多个点云同时显示并着色
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

char filepath1[] = "B:\\PCL\\PCDFILE\\table_scene_lms400.pcd";
char filepath2[] = "B:\\PCL\\PCDFILE\\pcl_logo.pcd";

int 
main() 
{
	pcl::PointCloud<pcl::PointNormal>::Ptr src1(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr src2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::io::loadPCDFile<pcl::PointNormal>(filepath1, *src1);
	pcl::io::loadPCDFile<pcl::PointNormal>(filepath2, *src2);

	pcl::visualization::PCLVisualizer viewer("mainwindow");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> model_cloud_color_handler(src1, 0, 0, 100);
	viewer.addPointCloud(src1, model_cloud_color_handler, "name1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> source_cloud_color_handler(src2, 0, 100, 0);
	viewer.addPointCloud(src2, source_cloud_color_handler, "name2");

	viewer.addCoordinateSystem(2);
	viewer.initCameraParameters();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}