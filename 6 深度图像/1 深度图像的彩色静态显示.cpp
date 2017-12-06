//深度图像头文件
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>                        //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                    //PCD文件读取
int
main(int argc, char** argv)
{
	//define 并读取
	pcl::RangeImage rangeImage; 
	pcl::io::loadPCDFile("B:\\PCL\\PCDFILE\\range.pcd", rangeImage);
	std::cout << rangeImage << "\n";
	//show the range image,深度值作为颜色显示(静态图不能更新)
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(rangeImage);

	while (!viewer.wasStopped());
	return 0;
}

