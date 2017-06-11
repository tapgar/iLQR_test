#pragma once

#include "Common_Structs.h"
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace OpenSim;
using namespace SimTK;
using namespace std;

class iLQR
{
public:
	iLQR(Model *pModel);
	~iLQR();

	void Run(Trajectory init_traj);

private:

	Model *m_pModel; //main arm sim model

	void RunForward();
	void RunBackward();

	void ResetModel();

};

