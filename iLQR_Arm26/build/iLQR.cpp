#include "iLQR.h"


using namespace OpenSim;
using namespace std;


iLQR::iLQR(Model* pModel, Vector4d targ_x, double sim_time, int time_steps)
{
	m_pModel = pModel;
	ResetModel();
	m_pController = new LTV_Controller(pModel, sim_time, time_steps);

	m_pController->setActuators(m_pModel->updActuators());
	m_pModel->addController(m_pController);

	m_dMaxSimTime_s = sim_time;
	m_nTrajPts = time_steps;

	target_x = targ_x;

	for (int i = 0; i < 4; i++)
		Qf(i, i) = 10.0;

	R = Matrix<double, 6, 6>::Zero();
	for (int i = 0; i < 6; i++)
		R(i, i) = 0.20;

	lx = Vector4d::Zero();
	lu = VectorXd::Zero(6);

	lxx = Matrix4d::Zero();
	luu = Matrix<double, 6, 6>::Zero();
	lux = Matrix<double, 6, 4>::Zero();

	for (int i = 0; i < time_steps; i++)
	{
		Matrix<double, 6, 4> K = Matrix<double, 6, 4>::Zero();
		m_K.push_back(K);
		Matrix<double, 6, 1> k = Matrix<double, 6, 1>::Zero();
		m_k.push_back(k);
	}

	m_pController->SetControllerGains(m_K, m_k);

	m_pDynamics = new DynamicModel(pModel);
	m_nNumIters = 0;
}


iLQR::~iLQR()
{
}

void iLQR::Run()
{
	static bool bConverged = false;

	vector<Traj_Pt> closed_traj, open_traj;
	Traj_Pt null_pt;
	for (int i = 0; i < m_nTrajPts; i++)
	{
		closed_traj.push_back(null_pt);
		open_traj.push_back(null_pt);
	}

	while (!bConverged)
	{

		RunForward(&closed_traj, &open_traj);
		/*		for (int i = 0; i < m_nTrajPts; i++)
		{
		cout << open_traj[i].x << endl;
		cout << endl;
		cout << open_traj[i].u << endl;
		cout << endl;
		}
		for (int i = 0; i < m_nTrajPts; i++)
		{
		cout << closed_traj[i].x << endl;
		cout << endl;
		cout << closed_traj[i].u << endl;
		cout << endl;
		}
		*/
		RunBackward(closed_traj, open_traj);

		if (m_nNumIters++ > 99)
			bConverged = true;

	}
}

void iLQR::RunForward(vector<Traj_Pt>* closed_traj, vector<Traj_Pt>* open_traj)
{
	m_pController->SetDesiredTrajectory((*closed_traj));
	RunModelOpenLoop(open_traj);
	m_pController->SetDesiredTrajectory((*open_traj));
	RunModel(closed_traj);
}

void iLQR::RunBackward(vector<Traj_Pt> closed_traj, vector<Traj_Pt> open_traj)
{
	int idx = m_nTrajPts - 1;

	vector<Matrix<double, 6, 1>> m_Qu;
	vector<Matrix<double, 6, 6>> m_Quu;
	vector<Matrix<double, 6, 4>> m_Qux;

	Matrix<double, 6, 1> Qu = -R*(open_traj[idx].u - closed_traj[idx].u);
	Matrix<double, 6, 6> Quu = R;
	Vector4d Qx = Qf*(target_x - closed_traj[idx].x);
	Matrix4d Qxx = Qf;
	Matrix<double, 6, 4> Qux = Matrix<double, 6, 4>::Zero();

	Vector4d Vx = Qx - m_K[idx].transpose()*Quu*m_k[idx];
	Matrix4d Vxx = Qxx - m_K[idx].transpose()*Quu*m_K[idx];

	m_Qu.push_back(Qu);
	m_Quu.push_back(Quu);
	m_Qux.push_back(Qux);

	for (int i = idx - 1; i >= 0; i--)
	{
		Matrix<double, 6, 1> u_bar = -(open_traj[i].u - closed_traj[i].u);
		double l = u_bar.transpose()*R*u_bar;
		lu = 2 * u_bar;
		luu = R;
		//no state based cost besides Qf so lx, lxu, and lxx are zero

		//cout << lu << endl; cin.get();
		//cout << luu << endl; cin.get();

		Matrix4d A = Matrix4d::Zero();
		Matrix<double, 4, 6> B = Matrix<double, 4, 6>::Zero();

		m_pDynamics->GetGradient(closed_traj[i], double(i)*m_dMaxSimTime_s / double(m_nTrajPts), &A, &B);

		Qx = lx + A.transpose()*Vx;
		Qu = lu + B.transpose()*Vx;
		Qxx = lxx + A.transpose()*Vxx*A;
		Qux = lux + B.transpose()*Vxx*A;
		Quu = luu + B.transpose()*Vxx*B;

		Vx = Qx - m_K[idx].transpose()*Quu*m_k[idx];
		Vxx = Qxx - m_K[idx].transpose()*Quu*m_K[idx];

		m_Qu.push_back(Qu);
		m_Quu.push_back(Quu);
		m_Qux.push_back(Qux);
	}

	for (int i = 0; i < m_nTrajPts; i++)
	{
		//these were pushed into the vector backwards
		Quu = m_Quu[m_nTrajPts - 1 - i];
		Qu = m_Qu[m_nTrajPts - 1 - i];
		Qux = m_Qux[m_nTrajPts - 1 - i];

		//cout << "Gains: " << i << endl;

		m_K[i] = -Quu.inverse()*Qux;
		cout << m_K[i] << endl;
		m_k[i] = -Quu.inverse()*Qu;
		cout << m_k[i] << endl;
	}

	//cin.get();

	m_pController->SetControllerGains(m_K, m_k);
}

void iLQR::ResetModel()
{
	// Initialize the system and get the state representing the state system
	SimTK::State& si = m_pModel->initSystem();

	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	coords.get(coord_names[0]).setValue(si, 0.0); //idk this was in some example
	coords.get(coord_names[1]).setValue(si, 0.785375); //idk this was in some example

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

	//	if (!m_pModel->isControlled())
	//		m_pModel->addController(m_pController);

	m_pController->EnableFeedback();

	SimTK::State& si = m_pModel->initializeState();
	m_pModel->getStateValues(si);

	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	coords.get(coord_names[0]).setValue(si, 0.0); //idk this was in some example
	coords.get(coord_names[1]).setValue(si, 0.785375); //idk this was in some example

													   // Set the initial muscle activations 
	const Set<Muscle> &muscleSet = m_pModel->getMuscles();
	for (int i = 0; i< muscleSet.getSize(); i++) {
		muscleSet[i].setActivation(si, 0.01);
	}

	// Make sure the muscles states are in equilibrium
	m_pModel->equilibrateMuscles(si);

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator
		integrator(m_pModel->getMultibodySystem());
	integrator.setAccuracy(1.0e-2);

	Manager manager(*m_pModel, integrator);

	// Integrate from initial time to final time.
	manager.setInitialTime(0.0);
	manager.setFinalTime(m_dMaxSimTime_s);

	manager.integrate(si);

	for (int i = 0; i < m_nTrajPts; i++)
	{
		Traj_Pt pt;
		double time_step = double(i)*m_dMaxSimTime_s / double(m_nTrajPts);
		ProcessStateStorage(manager.getStateStorage(), time_step, &pt.x, &pt.u);
		(*trajectory)[i] = pt;
	}
	char buff[50];
	sprintf(buff, "Arm26_closed_%d.sto", m_nNumIters);
	manager.getStateStorage().print(buff);
}

void iLQR::RunModelOpenLoop(vector<Traj_Pt>* trajectory)
{
	ResetModel();

	//	if (!m_pModel->isControlled())
	//		m_pModel->addController(m_pController);

	m_pController->DisableFeedback();

	SimTK::State& si = m_pModel->initializeState();
	m_pModel->getStateValues(si);

	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	coords.get(coord_names[0]).setValue(si, 0.0); //idk this was in some example
	coords.get(coord_names[1]).setValue(si, 0.785375); //idk this was in some example

													   // Set the initial muscle activations 
	const Set<Muscle> &muscleSet = m_pModel->getMuscles();
	for (int i = 0; i< muscleSet.getSize(); i++) {
		muscleSet[i].setActivation(si, 0.01);
	}

	// Make sure the muscles states are in equilibrium
	m_pModel->equilibrateMuscles(si);

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator
		integrator(m_pModel->getMultibodySystem());
	integrator.setAccuracy(1.0e-2);

	Manager manager(*m_pModel, integrator);

	// Integrate from initial time to final time.
	manager.setInitialTime(0.0);
	manager.setFinalTime(m_dMaxSimTime_s);

	manager.integrate(si);

	for (int i = 0; i < m_nTrajPts; i++)
	{
		Traj_Pt pt;
		double time_step = (double(i) + 0.5)*m_dMaxSimTime_s / double(m_nTrajPts);
		ProcessStateStorage(manager.getStateStorage(), time_step, &pt.x, &pt.u);
		(*trajectory)[i] = pt;
	}
	char buff[50];
	sprintf(buff, "Arm26_open_%d.sto", m_nNumIters);
	manager.getStateStorage().print(buff);
	//manager.getStateStorage().print("Arm26_states.sto");
}
