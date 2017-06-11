#include "LTV_Controller.h"
#include <OpenSim/OpenSim.h> //"OpenSim.h"

using namespace OpenSim;
using namespace SimTK;

void LTV_Controller::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
	//create a new OpenSim model
	//Model osimModel("Arm26_Optimize.osim");
	
	//initialize the system and get the state representing the state system
	//State& s = osimModel.initSystem();
	
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
	
	Muscle* TRIlong = dynamic_cast<Muscle*> (&getActuatorSet().get(0));
	Muscle* TRIlat = dynamic_cast<Muscle*> (&getActuatorSet().get(1));
	Muscle* TRImed = dynamic_cast<Muscle*> (&getActuatorSet().get(2));
	Muscle* BIClong = dynamic_cast<Muscle*> (&getActuatorSet().get(3));
	Muscle* BICshort = dynamic_cast<Muscle*> (&getActuatorSet().get(4));
	Muscle* BRA = dynamic_cast<Muscle*> (&getActuatorSet().get(5));

	//get current state from s
	//calculate muscle activation
	const Set<Muscle>& muscleSet = osimModel.getMuscles();
	
	//define initial muscle states
	ActivationFiberLengthMuscle* TRIlong = dynamic_cast<ActivationFiberLengthMuscle*>(&muscleSet.get(0));
	ActivationFiberLengthMuscle* TRIlat = dynamic_cast<ActivationFiberLengthMuscle*>(&muscleSet.get(1));
	ActivationFiberLengthMuscle* TRImed = dynamic_cast<ActivationFiberLengthMuscle*>(&muscleSet.get(2));
	ActivationFiberLengthMuscle* BIClong = dynamic_cast<ActivationFiberLengthMuscle*>(&muscleSet.get(3));
	ActivationFiberLengthMuscle* BICshort = dynamic_cast<ActivationFiberLengthMuscle*>(&muscleSet.get(4));
	ActivationFiberLengthMuscle* BRA = dynamic_cast<ActivationFiberLengthMuscle*>(&muscleSet.get(5));
	
	//get activation level at current state from s
	//for(int i=0; i < muscleSet.getSize(); i++) {
	//	muscleSet[i].getActivation(s);
	u_TRIlong -> muscleSet(0).getActivation(s);
	u_TRIlat -> muscleSet(1).getActivation(s);
	u_TRImed -> muscleSet(2).getActivation(s);
	u_BIClong -> muscleSet(3).getActivation(s);
	u_BICshort -> muscleSet(4).getActivation(s);
	u_BRA -> muscleSet(5).getActivation(s);
	
	//number of controls will equal the number of muscles in the model
	int numControls = osimModel.getNumControls();
	
	//convert u to Vector controls object
	double u[] = {u_TRIlong, u_TRIlat, u_TRImed, u_BIClong, u_BICshort, u_BRA};
	Vector controls(numControls, u);
	Vector lower_bounds(numControls, 0.01);
	Vector upper_bounds(numControls, 0.99);
	
	//u = u_ff + Kt(x_t - x) + kt
	
	
}
