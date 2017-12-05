NormalEstimation这个类
做了以下3件事
1.得到p的最近邻
2.计算p的表面法线n
3.检查法线的朝向，然后拨乱反正。
默认的视角是(0,0,0)，可以通过下面的方法来更改
setViewPoint (float vpx, float vpy, float vpz);
计算一个点的法线
computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature);
前面两个参数很好理解，plane_parameters包含了4个参数，前面三个是法线的(nx,ny,nz)坐标，加上一个 nc . p_plane (centroid here) + p的坐标，然后最后一个参数是曲率。
