#include "DynamicModel.h"



DynamicModel::DynamicModel(Model* pModel)
{
	m_pModel = pModel;
}


DynamicModel::~DynamicModel()
{
}

void DynamicModel::GetGradient(Traj_Pt pt, double time_s, Matrix4d* fx, Matrix<double, 4, 6>* fu)
{
	Matrix4d forwardA = Matrix4d::Zero();
	Matrix4d backwardA = Matrix4d::Zero();
	SimTK::State forS;
	SimTK::State backS;
	Vector4d forVec = Vector4d::Zero();
	Vector4d backVec = Vector4d::Zero();

	for (int i = 0; i < 4; i++)
	{
		Traj_Pt forwardPt = pt;
		forwardPt.x(i) += 0.01;
		ResetModelAtPoint(forwardPt, &forS);
		RunForward(time_s, forS, &forVec);

		Traj_Pt backwardPt = pt;
		backwardPt.x(i) -= 0.01;
		ResetModelAtPoint(backwardPt, &backS);
		RunForward(time_s, backS, &backVec);


		for (int j = 0; j < 4; j++)
		{
			(*fx)(j, i) = (forVec(j) - backVec(j)) / 0.02;
		}
	}

	for (int i = 0; i < 6; i++)
	{
		Traj_Pt forwardPt = pt;
		forwardPt.u(i) += 0.01;
		ResetModelAtPoint(forwardPt, &forS);
		RunForward(time_s, forS, &forVec);

		Traj_Pt backwardPt = pt;
		backwardPt.u(i) -= 0.01;
		ResetModelAtPoint(backwardPt, &backS);
		RunForward(time_s, backS, &backVec);


		for (int j = 0; j < 4; j++)
		{
			(*fu)(j, i) = (forVec(j) - backVec(j)) / 0.02;
		}
	}
}

void DynamicModel::RunForward(double time, SimTK::State sp, Vector4d* vec)
{
	SimTK::RungeKuttaMersonIntegrator
		integrator(m_pModel->getMultibodySystem());
	integrator.setAccuracy(1.0e-4);

	Manager manager((*m_pModel), integrator);

	// Integrate from initial time to final time.
	manager.setInitialTime(time);
	manager.setFinalTime(time + 0.01);

	sp.setTime(time);

	manager.integrate(sp);

	VectorXd controls = VectorXd::Zero(6);
	ProcessStateStorage(manager.getStateStorage(), time + 0.005, vec, &controls);

}

void DynamicModel::ResetModelAtPoint(Traj_Pt pt, SimTK::State* s)
{
	(*s) = m_pModel->initializeState();
	//m_pModel->getStateValues(*s);
	// initialize the starting shoulder angle
	const CoordinateSet& coords = m_pModel->getCoordinateSet();
	for (int i = 0; i < 2; i++)
	{
		coords.get(coord_names[i]).setValue(*s, pt.x(i));
		coords.get(coord_names[i]).setSpeedValue(*s, pt.x(i + 2));
	}

	// Set the initial muscle activations 
	const Set<Muscle> &muscleSet = m_pModel->getMuscles();
	for (int i = 0; i< muscleSet.getSize(); i++) {
		muscleSet[i].setActivation(*s, pt.u(i));
	}

	// Make sure the muscles states are in equilibrium
	// m_pModel->equilibrateMuscles(*s);
}