#include <pcl/impl/point_types.hpp>

typedef unsigned int uint;

uint DBSCAN(std::vector<pcl::PointXYZ>& database, std::vector<int>& point_label, float eps, uint min_pts);