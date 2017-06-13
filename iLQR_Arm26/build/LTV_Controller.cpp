#include "LTV_Controller.h"
#include <OpenSim/OpenSim.h> //"OpenSim.h"

using namespace OpenSim;
using namespace Eigen;

void LTV_Controller::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
	// Get the current time in the simulation.
	double t = s.getTime();

	int nT = int(t*double(m_nTimeSteps) / m_dMaxSimTime_s + 0.5);

	if (nT >= m_nTimeSteps)
		nT = m_nTimeSteps - 1;
	if (nT < 0)
		nT = 0;

	MatrixXd K = m_K[nT];
	VectorXd k = m_k[nT];

	Traj_Pt targ_pt = m_Trajectory[nT];

	//m_pModel->setStateValues(s);

	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	Vector4d curX = Vector4d::Zero();
	for (int i = 0; i < 2; i++)
	{
		curX(i) = coords.get(coord_names[i]).getValue(s);
		curX(i + 2) = coords.get(coord_names[i]).getSpeedValue(s);
	}

	VectorXd u = VectorXd::Zero(6);
	u = targ_pt.u + k;
	if (m_bFeedbackEnabled)
		u += K*(targ_pt.x - curX);

	for (int i = 0; i < 6; i++)
	{
		Muscle* tempMuscle = dynamic_cast<Muscle*>	(&getActuatorSet().get(i));
		SimTK::Vector muscleControl(1, u(i));
		// Add in the controls computed for this muscle to the set of all model controls
		tempMuscle->addInControls(muscleControl, controls);
	}
}
