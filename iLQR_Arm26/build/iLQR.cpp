#include "iLQR.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;


iLQR::iLQR(Model* pModel, double sim_time, int time_steps)
{
	m_pModel = pModel;
	ResetModel();
	m_pController = new LTV_Controller(sim_time, time_steps);
}


iLQR::~iLQR()
{
}

void iLQR::Run(vector<Traj_Pt> init_traj, Traj_Pt target)
{
	static bool bConverged = false;

	while (!bConverged)
	{
		RunForward();
		RunBackward();
	}
}

void iLQR::RunForward()
{
	
}

void iLQR::RunBackward()
{

}

void iLQR::ResetModel()
{
	// Initialize the system and get the state representing the state system
	State& si = m_pModel->initSystem();

	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	coords.get("r_shoulder_elev").setValue(si, -1.57079633); //idk this was in some example

	// Set the initial muscle activations 
	const Set<Muscle> &muscleSet = m_pModel->getMuscles();
	for (int i = 0; i< muscleSet.getSize(); i++) {
		muscleSet[i].setActivation(si, 0.01);
	}

	// Make sure the muscles states are in equilibrium
	m_pModel->equilibrateMuscles(si);
}

void iLQR::RunModel(vector<Traj_Pt>* trajectory)
{
	ResetModel();

	if (!m_pModel->isControlled())
		m_pModel->addController(m_pController);



}
