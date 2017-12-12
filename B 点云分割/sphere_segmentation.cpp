#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);

	// 生成数据，生成立方体模型
	cloud->width = 10000;cloud->height = 1;cloud->points.resize(cloud->width * cloud->height);
	std::vector<int> indexs;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		float px, py, pz;
		px = rand() / (RAND_MAX + 1.0f);
		py = rand() / (RAND_MAX + 1.0f);
		pz = rand() / (RAND_MAX + 1.0f);
		//if (px < 0.1 || py < 0.1 || pz < 0.1||px>0.9 || py>0.9 || pz>0.9) //加上这句为空心立方体
		{
			cloud->points[i].x = px;
			cloud->points[i].y = py;
			cloud->points[i].z = pz;
		}
	}

	//创建分割时所需要的模型系数对象（coefficients），存储内点的点索引集合对象（inliers）
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 可选择配置，设置模型系数需要优化
	seg.setOptimizeCoefficients(true);
	// 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
	seg.setModelType(pcl::SACMODEL_SPHERE);   //设置模型类型-球体
	seg.setMethodType(pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
	seg.setDistanceThreshold(0.1);    //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	//拷贝筛分割后剩下的点到cloudout
	pcl::copyPointCloud(*cloud, *inliers, *cloudout);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a  model for the given dataset.");
		return (-1);
	}
	 //X轴平移便于观察
	for (size_t i = 0; i < cloudout->points.size(); ++i)
	{
		cloudout->points[i].x = cloudout->points[i].x-2.0;
	}

	//可视化,分别上不同颜色
	pcl::visualization::PCLVisualizer viewer("main window");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, green_color, "cloud1");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(cloudout, 0,0,255);
	viewer.addPointCloud<pcl::PointXYZ>(cloudout, blue_color, "cloud2");

	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters(); 
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return (0);
}