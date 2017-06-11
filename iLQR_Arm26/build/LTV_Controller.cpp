#include "LTV_Controller.h"
#include <OpenSim/OpenSim.h> //"OpenSim.h"

using namespace OpenSim;
using namespace Eigen;

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

	m_pModel->getStateValues(s);
	
	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	Vector4d curX;
	for (int i = 0; i < 2; i++)
	{
		curX[i] = coords.get(i).getValue(s);
		curX[i + 2] = coords.get(i).getSpeedValue(s);
	}

	VectorXd u = targ_pt.u + K*(targ_pt.x - curX) + k;
	for (int i = 0; i < 6; i++)
		controls.set(i, u[i]);
}
