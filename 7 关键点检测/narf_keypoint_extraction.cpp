/* \author Bastian Steder */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

// -----Parameters-----
std::string filename = "e:\\pcl\\pcdfile\\table_scene_lms400.pcd";
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = true;

// -----Main-----
int
main(int argc, char** argv)
{
	// -----Parse Command Line Arguments-----
	angular_resolution = pcl::deg2rad(angular_resolution);
	// -----Read pcd file or create example point cloud if not given-----
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity()); //´«¸ÐÆ÷³¯Ïò

	if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
	{
		cerr << "Was not able to open file \"" << filename << "\".\n";
		return 0;
	}
	/*
	for (float x = -0.5f; x <= 0.5f; x += 0.01f)
	{
		for (float y = -0.5f; y <= 0.5f; y += 0.01f)
		{
			pcl::PointXYZ point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
			point_cloud.points.push_back(point);
		}
	}
	point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;
	*/


	scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
		point_cloud.sensor_origin_[1],
		point_cloud.sensor_origin_[2])) *
		Eigen::Affine3f(point_cloud.sensor_orientation_);

	// -----Create RangeImage from the PointCloud-----
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;

	range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	if (setUnseenToMaxRange)  range_image.setUnseenToMaxRange();

	// -----Open 3D viewer and add point cloud-----
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	viewer.initCameraParameters();

	// -----Show range image-----
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	// -----Extract NARF keypoints-----
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

	// -----Show keypoints in 3D viewer-----
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.points.size());
	for (size_t i = 0; i<keypoint_indices.points.size(); ++i)
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	// -----Main loop-----
	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();  // process GUI events
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
}
