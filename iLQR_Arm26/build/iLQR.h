#pragma once

#include "Common_Structs.h"
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <vector>
#include "LTV_Controller.h"
#include "DynamicModel.h"

using namespace OpenSim;
using namespace std;

class iLQR
{
public:
	iLQR(Model *pModel, Vector4d targ_x, double sim_time, int time_steps);
	~iLQR();

	void Run(vector<Traj_Pt> init_traj, Traj_Pt target);

private:

	Model *m_pModel; //main arm sim model
	LTV_Controller *m_pController;

	double m_dMaxSimTime_s;
	int m_nTrajPts;

	void RunForward(vector<Traj_Pt>* closed_traj, vector<Traj_Pt>* open_traj);
	void RunBackward(vector<Traj_Pt> closed_traj, vector<Traj_Pt> open_traj);

	void ResetModel();

	void RunModel(vector<Traj_Pt>* trajectory);
	void RunModelOpenLoop(vector<Traj_Pt>* trajectory);

	vector<MatrixXd> m_K;
	vector<VectorXd> m_k;

	Vector4d target_x;

	Matrix4d Qf;
	MatrixXd R;

	Vector4d lx;
	VectorXd lu;

	Matrix4d lxx;
	MatrixXd lux;
	MatrixXd luu;

	DynamicModel* m_pDynamics;

};

