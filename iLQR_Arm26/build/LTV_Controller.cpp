#include "LTV_Controller.h"

using namespace OpenSim;
using namespace SimTK;

void LTV_Controller::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
	// Get the current time in the simulation.
	double t = s.getTime();

	int nT = int(t*double(m_nTimeSteps)/ m_dMaxSimTime_s + 0.5);

	if (nT >= m_nTimeSteps)
		nT = m_nTimeSteps - 1;
	if (nT < 0)
		nT = 0;

	MatrixXd K = m_K[nT];
	VectorXd k = m_k[nT];

	Traj_Pt targ_pt = m_Trajectory[nT];
	

	//get current state from s
	//calculate muscle activation
	//u = u_ff + Kt(x_t - x) + kt
	//convert u to Vector controls object

	
}