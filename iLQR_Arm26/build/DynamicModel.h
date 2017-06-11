#pragma once

#include <OpenSim/OpenSim.h>
#include <vector>
#include "Common_Structs.h"

using namespace OpenSim;
using namespace std;

class DynamicModel
{
public:
	DynamicModel(Model *pModel);

	void GetGradient(Traj_Pt pt, double time_s, Matrix4d* fx, MatrixXd* fu);

	~DynamicModel();

private:

	Model m_Model;

	void RunForward(double time, SimTK::State sp, Vector4d* vec);

	void ResetModelAtPoint(Traj_Pt pt, SimTK::State* s);

};

