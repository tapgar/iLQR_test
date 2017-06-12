#include "iLQR.h"


using namespace OpenSim;
using namespace std;


iLQR::iLQR(Model* pModel, Vector4d targ_x, double sim_time, int time_steps)
{
	m_pModel = pModel;
	ResetModel();
	m_pController = new LTV_Controller(pModel, sim_time, time_steps);

	m_pController->setActuators(m_pModel->updActuators());

	m_dMaxSimTime_s = sim_time;
	m_nTrajPts = time_steps;

	target_x = targ_x;

	lx = Vector4d::Zero();
	lu = VectorXd::Zero(6);

	lxx = Matrix4d::Zero();
	luu = MatrixXd::Zero(6, 6);
	lux = MatrixXd::Zero(6, 4);

	for (int i = 0; i < time_steps; i++)
	{
		MatrixXd K = MatrixXd::Zero(6, 4);
		m_K.push_back(K);
		VectorXd k = VectorXd::Zero(6);
		m_k.push_back(k);
	}

	m_pDynamics = new DynamicModel(pModel);
}


iLQR::~iLQR()
{
}

void iLQR::Run(vector<Traj_Pt> init_traj, Traj_Pt target)
{
	static bool bConverged = false;

	while (!bConverged)
	{
		vector<Traj_Pt> closed_traj, open_traj;
		Traj_Pt null_pt;
		for (int i = 0; i < m_nTrajPts; i++)
		{
			closed_traj.push_back(null_pt);
			open_traj.push_back(null_pt);
		}
		RunForward(&closed_traj, &open_traj);
		RunBackward(closed_traj, open_traj);
	}
}

void iLQR::RunForward(vector<Traj_Pt>* closed_traj, vector<Traj_Pt>* open_traj)
{
	RunModelOpenLoop(open_traj);
	m_pController->SetDesiredTrajectory((*open_traj));
	RunModel(closed_traj);
}

void iLQR::RunBackward(vector<Traj_Pt> closed_traj, vector<Traj_Pt> open_traj)
{
	int idx = m_nTrajPts - 1;

	vector<VectorXd> m_Qu;
	vector<MatrixXd> m_Quu;
	vector<MatrixXd> m_Qux;
	
	VectorXd Qu = R*(open_traj[idx].u - closed_traj[idx].u);
	MatrixXd Quu = R;
	Vector4d Qx = Qf*(target_x - closed_traj[idx].x);
	Matrix4d Qxx = Qf;
	MatrixXd Qux = MatrixXd::Zero();

	Vector4d Vx = Qx - m_K[idx].transpose()*Quu*m_k[idx];
	Matrix4d Vxx = Qxx - m_K[idx].transpose()*Quu*m_K[idx];

	m_Qu.push_back(Qu);
	m_Quu.push_back(Quu);
	m_Qux.push_back(Qux);

	for (int i = idx - 1; i >= 0; i--)
	{
		VectorXd u_bar = (open_traj[i].u - closed_traj[i].u);
		double l = u_bar.transpose()*R*u_bar;
		lu = 2 * u_bar;
		luu = R;
		//no state based cost besides Qf so lx, lxu, and lxx are zero

		Matrix4d A = Matrix4d::Zero();
		MatrixXd B = MatrixXd::Zero(4, 6);

		m_pDynamics->GetGradient(closed_traj[i], double(i) / 100.0, &A, &B);

		Qx = lx + A.transpose()*Vx;
		Qu = lu + B.transpose()*Vx;
		Qxx = lxx + A.transpose()*Vxx*A;
		Qux = lux + B.transpose()*Vxx*B;
		Quu = luu + B.transpose()*Vxx*B;

		Vx = Qx - m_K[idx].transpose()*Quu*m_k[idx];
		Vxx = Qxx - m_K[idx].transpose()*Quu*m_K[idx];

		m_Qu.push_back(Qu);
		m_Quu.push_back(Quu);
		m_Qux.push_back(Qux);
	}

	for (int i = 0; i < m_nTrajPts; i++)
	{
		Quu = m_Quu[i];
		Qu = m_Qu[i];
		Qux = m_Qux[i];

		m_K[i] = -Quu.inverse()*Qux;
		m_k[i] = -Quu.inverse()*Qu;
	}

//	m_pController->SetControllerGains(K, k);
}

void iLQR::ResetModel()
{
	// Initialize the system and get the state representing the state system
	SimTK::State& si = m_pModel->initSystem();

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

	m_pController->EnableFeedback();

	SimTK::State si;
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

void iLQR::RunModelOpenLoop(vector<Traj_Pt>* trajectory)
{
	ResetModel();

	if (!m_pModel->isControlled())
		m_pModel->addController(m_pController);

	m_pController->DisableFeedback();

	SimTK::State si;
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
