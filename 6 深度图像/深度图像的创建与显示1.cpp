//���ͼ��ͷ�ļ�
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

int 
main(int argc, char** argv) 
{
	//define VAR
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::RangeImage rangeImage;
	// Generate the data--һ��ƽ��
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

	// �ӵ��ƴ���PointCloud--->RangeImage rangeImage;
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;
	
	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << rangeImage << "\n";

	//show the point cloud
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(pointCloud.makeShared()); //ָ���ʽת��
	
	//show the range image,���ֵ��Ϊ��ɫ��ʾ(��̬ͼ���ܸ���)
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(rangeImage);

	while (!viewer.wasStopped()); 
	return 0;
}
