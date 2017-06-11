#include "iLQR.h"



using namespace OpenSim;
using namespace SimTK;
using namespace std;


iLQR::iLQR(Model* pModel)
{
	m_pModel = pModel;
	ResetModel();
}


iLQR::~iLQR()
{
}

void iLQR::Run(Trajectory init_traj)
{
	static bool bConverged = false;

	while (!bConverged)
	{
		RunForward();
		RunBackward();
		ResetModel();
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
