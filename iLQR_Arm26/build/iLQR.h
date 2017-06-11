#pragma once

#include "Common_Structs.h"
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <vector>
#include "LTV_Controller.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

class iLQR
{
public:
	iLQR(Model *pModel, double sim_time, int time_steps);
	~iLQR();

	void Run(vector<Traj_Pt> init_traj, Traj_Pt target);

private:

	Model *m_pModel; //main arm sim model
	LTV_Controller *m_pController;

	void RunForward();
	void RunBackward();

	void ResetModel();

	void RunModel(vector<Traj_Pt>* trajectory);

};

