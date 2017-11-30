/* \author Bastian Steder */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ My_Point;

// -----Parameters-----
float angular_resolution = 0.5f; //角坐标分辨率
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//规定坐标系方向
bool setUnseenToMaxRange = false; //不可见区域处理

// -----Main-----
int
main(int argc, char** argv)
{
	angular_resolution = pcl::deg2rad(angular_resolution);

	pcl::PointCloud<My_Point>::Ptr point_cloud_ptr(new pcl::PointCloud<My_Point>);
	pcl::PointCloud<My_Point>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

	//creat data
	for (float x = -0.5f; x <= 0.5f; x += 0.01f)
	{
		for (float y = -0.5f; y <= 0.5f; y += 0.01f)
		{
			My_Point point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
			point_cloud.points.push_back(point);
		}
	}
	point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;
	
	// -----Create RangeImage from the PointCloud-----
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)  range_image.setUnseenToMaxRange();

	// -----Open 3D viewer and add point cloud-----
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0f, "global");
	pcl::visualization::PointCloudColorHandlerCustom<My_Point> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
	viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");

	// -----Extract borders-----
	pcl::RangeImageBorderExtractor border_extractor(&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);


	// -----Show points on range image-----
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	range_image_borders_widget =pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false,
			border_descriptions, "Range image with borders");
	
	// -----Main loop-----
	while (!viewer.wasStopped())
	{
		range_image_borders_widget->spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
}
