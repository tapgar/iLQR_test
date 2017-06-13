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
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	iLQR(Model *pModel, Vector4d targ_x, double sim_time, int time_steps);
	~iLQR();

	void Run();

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
	Matrix<double, 6, 6, DontAlign> R;

	Vector4d lx;
	Matrix<double, 6, 1, DontAlign> lu;

	Matrix4d lxx;
	Matrix<double, 6, 4, DontAlign> lux;
	Matrix<double, 6, 6, DontAlign> luu;

	DynamicModel* m_pDynamics;

	int m_nNumIters;

	double m_dLastCost;
	double m_dLambda;

};

