#include <vector>
#include <pcl/impl/point_types.hpp>
#include <iostream>

struct OBB_Descriptor {
	inline OBB_Descriptor() :center_x(0), center_y(0), center_z(0), main_vector_x(1), main_vector_y(0) {}
	float center_x;
	float center_y;
	float center_z;
	float main_vector_x; //主轴朝向
	float main_vector_y;
	void print() {
		std::cout << "center = ( " << center_x << ", " << center_y << ", " << center_z << " )" << std::endl;
		std::cout << "axis = ( " << main_vector_x << ", " << main_vector_y << " )" << std::endl;
	}
};

class OBB {
public:
	OBB(std::vector<pcl::PointXYZ> input) : database(input) {};
	void compute(OBB_Descriptor& output);
private:
	std::vector<pcl::PointXYZ> database;
};
