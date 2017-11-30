#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//定义点云
pcl::PointCloud<pcl::PointXYZ>::Ptr 	basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 	point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

int testtype = 6;
//simpleVis函数实现最基本的点云PointXYZ可视化操作，
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
	viewer->setBackgroundColor(0, 0, 0);
	/*将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能标志引用该点云，多次调用addPointCloud
	可以实现多个点云的添加，，每调用一次就会创建一个新的ID号，如果想更新一个已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号*/
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud"); //basic_cloud_ptr
	//用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	/*坐标轴 X（红色）Y（绿色 ）Z （蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0*/
	viewer->addCoordinateSystem(1.0);
	//通过设置照相机参数使得从默认的角度和方向观察点云
	viewer->initCameraParameters();
	return (viewer);
}

//带有RGB数据的属性字段的点云PointXYZRGB可视化
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	/*创建一个颜色处理对象，PointCloudColorHandlerRGBField利用这样的对象显示自定义颜色数据，PointCloudColorHandlerRGBField
	对象得到每个点云的RGB颜色字段*/
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	return (viewer);
}

//点云着上单独的一种颜色，可以利用该技术给指定的点云着色，以区别其他的点云，
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	//addPointCloud<>()完成对颜色处理器对象的传递
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

//normalsVis函数中演示了如何实现点云的法线，
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//实现对点云法线的显示
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

//绘制普通形状
/**************************************************************************************************************
PCL visualizer可视化类允许用户在视窗中绘制一般图元，这个类常用于显示点云处理算法的可视化结果，例如 通过可视化球体
包围聚类得到的点云集以显示聚类结果，shapesVis函数用于实现添加形状到视窗中，添加了四种形状：从点云中的一个点到最后一个点
之间的连线，原点所在的平面，以点云中第一个点为中心的球体，沿Y轴的椎体
*************************************************************************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],cloud->points[cloud->size() - 1], "line");
	//添加点云中第一个点为中心，半径为0.2的球体，同时可以自定义颜色
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");
	//Add shapes at other locations添加绘制平面使用标准平面方程ax+by+cz+d=0来定义平面，这个平面以原点为中心，方向沿着Z方向-----
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");
	//添加锥形的参数
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return (viewer);
}
/******************************************************************************************
多视角显示：PCL  visealizer可视化类允许用户通过不同的窗口（Viewport）绘制多个点云这样方便对点云比较
viewportsVis函数演示如何用多视角来显示点云计算法线的方法结果对比
******************************************************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	//以上是创建视图的标准代码
	int v1(0);  //创建新的视口
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，Y轴的最小值，	以及X、Y最大值，取值0-1，v1是标识
	viewer->setBackgroundColor(0, 0, 0, v1);    //设置视口的背景颜色
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
	//对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
	//为所有视口设置属性，
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
	//添加法线  每个视图都有一组对应的法线
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);
	return (viewer);
}
/*******************************************************************************************************
这里是处理鼠标事件的函数，每次相应鼠标时间都会回电函数，需要从event实例提取事件信息，本例中查找鼠标左键的释放事件
每次响应这种事件都会在鼠标按下的位置上生成一个文本标签。
*********************************************************************************************************/
unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}
/********************************************************************************************
键盘事件 我们按下哪个按键  如果按下r健   则删除前面鼠标所产生的文本标签，需要注意的是，当按下R键时 3D相机仍然会重置
所以在PCL中视窗中注册事件响应回调函数，不会覆盖其他成员对同一事件的响应
**************************************************************************************************/
void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

/******************自定义交互*****************************************************************************/
/******************************************************************************************************
多数情况下，默认的鼠标和键盘交互设置不能满足用户的需求，用户想扩展函数的某一些功能，  比如按下键盘时保存点云的信息，
或者通过鼠标确定点云的位置   interactionCustomizationVis函数进行演示如何捕捉鼠标和键盘事件，在窗口点击，将会显示
一个2D的文本标签，按下r健出去文本
******************************************************************************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	//以上是实例化视窗的标准代码
	viewer->addCoordinateSystem(1.0);
	//分别注册响应键盘和鼠标事件，keyboardEventOccurred  mouseEventOccurred回调函数，需要将boost::shared_ptr强制转换为void*
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());

	return (viewer);
}
//利用函数生成一个点云数据----------------------------------------------------------------------------
void creat_pcd(void)
{
	// We're going to make an ellipse extruded along the z-axis. The colour for the XYZRGB cloud will gradually go from red to green to blue.
	std::cout << "Genarating example point clouds.\n\n";
	uint8_t r(0), g(50), b(50);
	for (float z(-1.0); z <= 1.0; z += 0.05)//40 steps
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)//72 steps
		{
			pcl::PointXYZ point1;//普通点云
			point1.x = 0.5*cosf(pcl::deg2rad(angle)); //degree to radian
			point1.y = sinf(pcl::deg2rad(angle));
			point1.z = z;
			basic_cloud_ptr->points.push_back(point1);

			pcl::PointXYZRGB point2;//有色点云
			point2.x = point1.x;
			point2.y = point1.y;
			point2.z = point1.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point2.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point2);
		}
		r = (1 + z) * 255/2; //改变颜色信息

	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
}


//主函数------------------------------------------------------------------
int
main(int argc, char** argv)
{
	//生成点云函数
	creat_pcd();
	// Calculate surface normals with a search radius of 0.05 and 1.0
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

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (testtype == 0)
	{
		std::cout << "Simple visualisation example\n";
		viewer = simpleVis(basic_cloud_ptr);
	}
	else if (testtype == 1)
	{
		std::cout << "Custom colour visualisation example\n";
		viewer = rgbVis(point_cloud_ptr);
	}
	else if (testtype == 2)
	{
		std::cout << "Normals visualisation example\n";
		viewer = customColourVis(basic_cloud_ptr);
	}
	else if (testtype == 3)
	{
		std::cout << "Shapes visualisation example\n";
		viewer = normalsVis(point_cloud_ptr, cloud_normals2);
	}
	else if (testtype == 4)
	{
		std::cout << "Viewports example\n";
		viewer = shapesVis(point_cloud_ptr);
	}
	else if (testtype == 5)
	{
		std::cout << "viewportsVis example\n";
		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
	}
	else if (testtype == 6)
	{
		std::cout << "Interaction Customization example\n";
		viewer = interactionCustomizationVis();
	}
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}