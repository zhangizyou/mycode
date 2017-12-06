/* \author Bastian Steder */
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

int
main(void)
{
	
	pcl::RangeImage range_image;
	pcl::io::loadPCDFile("B:\\PCL\\PCDFILE\\range.pcd", range_image);

	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	range_image.setUnseenToMaxRange();

	// -----Open 3D viewer and add point cloud-----
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0f, "global");

	// -----Extract borders-----
	pcl::RangeImageBorderExtractor border_extractor(&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);

	// -----Show points on range image-----
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	range_image_borders_widget = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false,
		border_descriptions, "Range image with borders");

	// -----Main loop-----
	while (!viewer.wasStopped())
	{
		range_image_borders_widget->spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
}
