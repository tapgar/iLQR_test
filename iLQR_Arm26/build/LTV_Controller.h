#pragma once
#include <OpenSim/OpenSim.h>
#include <vector>
#include "Common_Structs.h"
#include <Eigen/Dense>

using namespace OpenSim;
using namespace std;

class LTV_Controller : public Controller 
{
	OpenSim_DECLARE_CONCRETE_OBJECT(LTV_Controller, Controller);

public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LTV_Controller(Model* pModel, double sim_time, int time_steps) : Controller() {
		m_dMaxSimTime_s = sim_time;
		m_nTimeSteps = time_steps;
		m_bFeedbackEnabled = true;
		m_pModel = pModel;
	};
	~LTV_Controller() { };

	void computeControls(const SimTK::State& s, SimTK::Vector &controls) const;

	void SetDesiredTrajectory(vector<Traj_Pt> traj_pts)
	{
		m_Trajectory = traj_pts;
	};

	void SetControllerGains(vector<MatrixXd> K, vector<VectorXd> k)
	{
		m_K = K;
		m_k = k;
	}

	void DisableFeedback() { m_bFeedbackEnabled = false; }
	void EnableFeedback() { m_bFeedbackEnabled = true; }

private:

	double m_dMaxSimTime_s;
	int m_nTimeSteps;

	bool m_bFeedbackEnabled;

	vector<Traj_Pt> m_Trajectory;

	vector<MatrixXd> m_K;
	vector<VectorXd> m_k;

	Model *m_pModel;
};

