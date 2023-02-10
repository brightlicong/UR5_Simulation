#include "OBB.h"
#include <Eigen/Dense>
using Eigen::MatrixX2f;
using Eigen::Matrix2f;

void OBB::compute(OBB_Descriptor& output) {
	size_t point_num = database.size();
	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;
	for (size_t i = 0; i < point_num; i++) {
		sum_x += database[i].x;
		sum_y += database[i].y;
		sum_z += database[i].z;
	}
	output.center_x = sum_x / point_num;
	output.center_y = sum_y / point_num;
	output.center_z = sum_z / point_num;
	// PCA
	float cov_xx = 0;
	float cov_xy = 0;
	float cov_yy = 0;
	for (size_t i = 0; i < point_num; i++) {
		float dx = database[i].x - output.center_x;
		float dy = database[i].y - output.center_y;
		cov_xx += dx * dx;
		cov_xy += dx * dy;
		cov_yy += dy * dy;
	}
	Eigen::Matrix2f covMat;
	point_num--; //为计算协方差所使用
	covMat(0, 0) = cov_xx / point_num;
	covMat(1, 0) = cov_xy / point_num;
	covMat(0, 1) = covMat(1, 0);
	covMat(1, 1) = cov_yy / point_num;
	std::cout << covMat << std::endl;
	Eigen::EigenSolver<Eigen::Matrix2f> es(covMat);
	Eigen::Matrix2f val = es.pseudoEigenvalueMatrix();
	Eigen::Matrix2f vec = es.pseudoEigenvectors();
	if (val(0, 0) > val(1, 1)) {
		output.main_vector_x = vec(0, 0);
		output.main_vector_y = vec(1, 0);
	}
	else {
		output.main_vector_x = vec(0, 1);
		output.main_vector_y = vec(1, 1);
	}
}