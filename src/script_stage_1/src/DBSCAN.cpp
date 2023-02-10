#include <vector>
#include "DBSCAN.h"

float distance(pcl::PointXYZ& p, pcl::PointXYZ& q) {
    return std::abs(p.x - q.x) + std::abs(p.y - q.y);
}

uint findNeighbourhood(std::vector<pcl::PointXYZ>& database, size_t p_index, std::vector<size_t>& neighbourhood_idx, float eps) {
    neighbourhood_idx.clear();
    uint neighbourhood_num = 0;
    size_t database_size = database.size();
    pcl::PointXYZ p = database[p_index];
    for (size_t i = 0; i < database_size; i++) {
        float dis = distance(p, database[i]);
        if (dis <= eps) {
            neighbourhood_num++;
            if (i != p_index) {
                neighbourhood_idx.push_back(i);
            }
        }
    }
    return neighbourhood_num;
}

uint DBSCAN(std::vector<pcl::PointXYZ> &database, std::vector<int> &point_label, float eps, uint min_pts) {
    uint group_num = 0;
    size_t database_size = database.size();
    //std::vector<bool> is_visited = std::vector<bool>(database_size, false);
    point_label.clear();
    point_label = std::vector<int>(database_size, 0);
    for (size_t i = 0; i < database_size; i++) {
        if (point_label[i]) {
            continue;
        }
        std::vector<size_t> neighbourhood_idx{};
        uint neighbourhood_num = findNeighbourhood(database, i, neighbourhood_idx, eps);
        if (neighbourhood_num < min_pts) {
            point_label[i] = -1;
            continue;
        }
        group_num++;
        point_label[i] = group_num;
        for (size_t j = 0; j < neighbourhood_idx.size(); j++) {
            size_t temp_point_idx = neighbourhood_idx[j];
            if (point_label[temp_point_idx] < 0) {
                point_label[temp_point_idx] = group_num;
            }
            if (point_label[temp_point_idx]) {
                continue;
            }
            point_label[temp_point_idx] = group_num;
            std::vector<size_t> placeholder{};
            neighbourhood_num = findNeighbourhood(database, temp_point_idx, placeholder, eps);
            if (neighbourhood_num > min_pts) {
                neighbourhood_idx.push_back(temp_point_idx);
            }

        }
    }
    return group_num;
}