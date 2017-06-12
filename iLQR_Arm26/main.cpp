#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include "iLQR.h"
#include "Common_Structs.h"

using namespace Eigen;
using namespace OpenSim;
//using namespace SimTK;
using namespace std;

int main()
{
	try {
		std::clock_t startTime = std::clock();

		// Create a new OpenSim model
		// Similar to arm26 model but without wrapping surfaces for better performance
		Model osimModel("Arm26_Optimize.osim");

		Vector4d targ_x = Vector4d::Zero();
		targ_x(0) = 1.116;
		targ_x(1) = 1.57;

		iLQR traj_opt = iLQR(&osimModel, targ_x, 0.5, 50);
		traj_opt.Run();

		cout << "OpenSim example completed successfully.\n";

		cin.get();


	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
		return 1;
	}

	// End of main() routine.
	return 0;
}