NormalEstimation�����
��������3����
1.�õ�p�������
2.����p�ı��淨��n
3.��鷨�ߵĳ���Ȼ���ҷ�����
Ĭ�ϵ��ӽ���(0,0,0)������ͨ������ķ���������
setViewPoint (float vpx, float vpy, float vpz);
����һ����ķ���
computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature);
ǰ�����������ܺ���⣬plane_parameters������4��������ǰ�������Ƿ��ߵ�(nx,ny,nz)���꣬����һ�� nc . p_plane (centroid here) + p�����꣬Ȼ�����һ�����������ʡ�
