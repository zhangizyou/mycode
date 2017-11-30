#include <iostream>                           //标准输入输出头文件申明
#include <pcl/io/io.h>                        //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                    //PCD文件读取
using namespace std;
int
main()
{

	pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);    //智能指针声明cloud 

	if(pcl::io::loadPCDFile("E:\\PCL\\PCDFILE\\8570.pcd", cloud)==-1)        //加载点云文件
	{
		PCL_ERROR("Can not open the file!\n");
		return(-1);
	}

	cout << "width=" << cloud.width << endl;
	cout << "height=" << cloud.height << endl;
	cout << "x0=" << cloud[0].x << endl;
	cout << "y0=" << cloud[0].y << endl;
	cout << "z0=" << cloud[0].z << endl;
	return 0;
}