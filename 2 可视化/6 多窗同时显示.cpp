#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
//�������---------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr 	basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 	point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

void creat_pcd(void);
//������--------------------------------------------------------------------------
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

	int v1(0);  //�����µ��ӿ�
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4�������ֱ���X��Y�����Сֵ���Լ�X��Y���ֵ��ȡֵ0-1��v1�Ǳ�ʶ
	viewer->setBackgroundColor(0, 0, 0, v1);    //�����ӿڵı�����ɫ
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //���һ����ǩ������������  ����RGB��ɫ��ɫ������ӵ��Ƶ��ӿ���
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud1", v1);
	//�Եڶ��ӿ���ͬ���Ĳ�����ʹ���������ĵ��Ʒֲ����Ұ봰�ڣ������ӿڱ�����ֵ�ڻ�ɫ���Ա�����������Ȼ���ͬ���ĵ��ƣ��������Զ�����ɫ��ɫ
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(point_cloud_ptr, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, single_color, "sample cloud2", v2);
	//Ϊ�����ӿ��������ԣ�
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
	//��ӷ���  ÿ����ͼ����һ���Ӧ�ķ���
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(point_cloud_ptr, cloud_normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(point_cloud_ptr, cloud_normals2, 10, 0.05, "normals2", v2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	return (0);
}

//���ú�������һ����������------------------------------------------------------
void creat_pcd(void)
{
	std::cout << "Genarating example point clouds.\n\n";
	uint8_t r(0), g(100), b(100);
	for (float z(-1.0); z <= 1.0; z += 0.05)//40 steps
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)//72 steps
		{
			pcl::PointXYZ point1;//��ͨ����
			point1.x = 0.5*cosf(pcl::deg2rad(angle)); //��Բ������degree to radian
			point1.y = sinf(pcl::deg2rad(angle));
			point1.z = z;
			basic_cloud_ptr->points.push_back(point1);

			pcl::PointXYZRGB point2;//��ɫ����
			point2.x = point1.x;
			point2.y = point1.y;
			point2.z = point1.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point2.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point2);
		}
		r = (1 + z) * 255 / 2; //�ı���ɫ��Ϣ

	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
}