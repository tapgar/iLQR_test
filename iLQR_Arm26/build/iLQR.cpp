#include "iLQR.h"


using namespace OpenSim;
using namespace SimTK;
using namespace std;


iLQR::iLQR(Model* pModel, double sim_time, int time_steps)
{
	m_pModel = pModel;
	ResetModel();
	m_pController = new LTV_Controller(sim_time, time_steps);

	m_pController->setActuators(m_pModel->updActuators());

	m_dMaxSimTime_s = sim_time;
	m_nTrajPts = time_steps;
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
	vector<Traj_Pt> traj;
	Traj_Pt null_pt;
	for (int i = 0; i < m_nTrajPts; i++)
		traj.push_back(null_pt); //i guess if we wanted to seperately use feedforward u we could do it here

	RunModel(&traj);


}

void iLQR::RunBackward()
{
	

//	m_pController->SetControllerGains(K, k);
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

	State si;
	m_pModel->getStateValues(si);
	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator
		integrator(m_pModel->getMultibodySystem());
	integrator.setAccuracy(1.0e-4);

	Manager manager(*m_pModel, integrator);

	// Integrate from initial time to final time.
	manager.setInitialTime(0.0);
	manager.setFinalTime(m_dMaxSimTime_s);
	
	manager.integrate(si);

	Storage stateStorage = manager.getStateStorage();
	
	//put data into trajectory object (memory will be allocated in trajectory so you can just do trajectory[x] = Traj_Pt;
}
