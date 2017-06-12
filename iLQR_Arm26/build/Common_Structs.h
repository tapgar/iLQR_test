#pragma once
#include <Eigen/Dense>
#include <vector>
#include "string.h"
#include <OpenSim/OpenSim.h>

using namespace Eigen;
using namespace std;
//using namespace OpenSim;

struct Traj_Pt {

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vector4d x;
	VectorXd u;
	Traj_Pt()
	{
		x = Vector4d::Zero();
		u = VectorXd::Zero(6);
	}
};

static const string coord_names[] = { "r_shoulder_elev", "r_elbow_flex" };
static const string muscle_names[] = { "TRIlong", "TRIlat", "TRImed", "BIClong", "BICshort", "BRA" };

static void ProcessStateStorage(OpenSim::Storage states, double time_lookup, Vector4d* state_vec, VectorXd* control_vec)
{
	double stateArray[16];
	//OpenSim::Array<double> idk(16);
	//states.print("Arm26_states.sto");
	//states.getDataAtTime(time_lookup, 16, idk);
	states.getDataAtTime(time_lookup, 16, stateArray);
	for (int i = 0; i < 4; i++)
	{
		(*state_vec)(i) = stateArray[i];
		//	printf("%f\t", (*state_vec)(i));
	}
	//printf("\n");

	for (int i = 0; i < 6; i++)
	{
		(*control_vec)(i) = stateArray[i * 2 + 4];
		//printf("%f\t", (*control_vec)(i));
	}
	//printf("\n");



}