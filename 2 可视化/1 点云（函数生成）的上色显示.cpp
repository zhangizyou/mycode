#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
//定义点云---------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr 	basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

void creat_pcd(void);
//主函数--------------------------------------------------------------------------
int
main()
{
	creat_pcd();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(basic_cloud_ptr, 0, 255, 0);
	//addPointCloud<>()完成对颜色处理器对象的传递
	viewer->addPointCloud<pcl::PointXYZ>(basic_cloud_ptr, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return (0);
}

//利用函数生成一个点云数据------------------------------------------------------
void creat_pcd(void)
{
	std::cout << "Genarating example point clouds.\n\n";
	uint8_t r(0), g(100), b(100);
	for (float z(-1.0); z <= 1.0; z += 0.05)//40 steps
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)//72 steps
		{
			pcl::PointXYZ point1;//普通点云
			point1.x = 0.5*cosf(pcl::deg2rad(angle)); //椭圆函数，degree to radian
			point1.y = sinf(pcl::deg2rad(angle));
			point1.z = z;
			basic_cloud_ptr->points.push_back(point1);
		}


	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;

}