//深度图像头文件
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int
main(int argc, char** argv)
{
	//define VAR
	pcl::PointCloud<pcl::PointXYZ> pointCloud; //普通点云
	pcl::RangeImage rangeImage; //深度图像

	// Generate the Points(一个平面)
	for (float y = -0.5f; y <= 0.5f; y += 0.01f) { //100
		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {//100
			pcl::PointXYZ point;
			point.x = 2.0f - y;
			point.y = y;
			point.z = z;
			pointCloud.points.push_back(point);
		}
	}
	pointCloud.width = (uint32_t)pointCloud.points.size();
	pointCloud.height = 1;

	// 从点云创建PointCloud--->RangeImage rangeImage;
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//设置传感器角度
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; //成像原理
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	std::cout << rangeImage << "\n";//显示创建信息
	pcl::io::savePCDFileASCII("range.pcd", rangeImage);//保存结果

	//show the point cloud
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(pointCloud.makeShared()); //指针格式转换

	while (!viewer.wasStopped());
	return 0;
}
