#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
//定义点云---------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr 	basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 	point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

void creat_pcd(void);
//主函数--------------------------------------------------------------------------
int
main()
{
	creat_pcd();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	int v1(0);  //创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X、Y轴的最小值，以及X、Y最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1);    //设置视口的背景颜色
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud1", v1);
	//对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(point_cloud_ptr, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, single_color, "sample cloud2", v2);
	//为所有视口设置属性，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
	//添加法线  每个视图都有一组对应的法线
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(point_cloud_ptr, cloud_normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(point_cloud_ptr, cloud_normals2, 10, 0.05, "normals2", v2);

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

			pcl::PointXYZRGB point2;//有色点云
			point2.x = point1.x;
			point2.y = point1.y;
			point2.z = point1.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point2.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point2);
		}
		r = (1 + z) * 255 / 2; //改变颜色信息

	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
}