//���ͼ��ͷ�ļ�
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>                        //I/O���ͷ�ļ�����
#include <pcl/io/pcd_io.h>                    //PCD�ļ���ȡ
int
main(int argc, char** argv)
{
	//define ����ȡ
	pcl::RangeImage rangeImage; 
	pcl::io::loadPCDFile("B:\\PCL\\PCDFILE\\range.pcd", rangeImage);
	std::cout << rangeImage << "\n";
	//show the range image,���ֵ��Ϊ��ɫ��ʾ(��̬ͼ���ܸ���)
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(rangeImage);

	while (!viewer.wasStopped());
	return 0;
}

