#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// -----Parameters-----
std::string filename ="e:\\pcl\\pcdfile\\room_scan1.pcd";
float angular_resolution_x = 0.5f,angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool live_update = false;


void
setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

// -----Main-----
int
main(int argc, char** argv)
{
	// -----Parse Command Line Arguments-----
	live_update = true;
	int tmp_coordinate_frame;
	coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
	angular_resolution_x = pcl::deg2rad(angular_resolution_x);
	angular_resolution_y = pcl::deg2rad(angular_resolution_y);

	// ------------------------------------------------------------------
	// -----Read pcd file or create example point cloud if not given-----
	// ------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

	if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
	{
		std::cout << "Was not able to open file \"" << filename << "\".\n";
		return 0;
	}
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
	range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
		pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	// -----Open 3D viewer and add point cloud-----
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

	viewer.initCameraParameters();
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());

	// -----Show range image-----
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	// -----Main loop-----
	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);

		if (live_update)
		{
			scene_sensor_pose = viewer.getViewerPose();
			range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
				pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
				scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
			range_image_widget.showRangeImage(range_image);
		}
	}
}
