#pragma once
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

struct Traj_Pt {
	Vector4d x;
	VectorXd u;
	Traj_Pt()
	{
		x = Vector4d::Zero();
		u = VectorXd::Zero(6);
	}
};

