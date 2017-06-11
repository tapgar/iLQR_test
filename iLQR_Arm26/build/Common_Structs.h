#pragma once
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

typedef struct {
	Vector4d x0;
	vector<VectorXd> u;
} Trajectory;