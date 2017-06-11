#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC


using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main()
{
	try {
		std::clock_t startTime = std::clock();

		// Create a new OpenSim model
		// Similar to arm26 model but without wrapping surfaces for better performance
		Model osimModel("Arm26_Optimize.osim");

		// Initialize the system and get the state representing the state system
		State& si = osimModel.initSystem();

		// initialize the starting shoulder angle
		const CoordinateSet& coords = osimModel.getCoordinateSet();
		coords.get("r_shoulder_elev").setValue(si, -1.57079633);

		// Set the initial muscle activations 
		const Set<Muscle> &muscleSet = osimModel.getMuscles();
		for (int i = 0; i< muscleSet.getSize(); i++) {
			muscleSet[i].setActivation(si, 0.01);
		}

		// Make sure the muscles states are in equilibrium
		osimModel.equilibrateMuscles(si);

		// The number of controls will equal the number of muscles in the model!
		int numControls = osimModel.getNumControls();
		int numStates = osimModel.getNumJoints();
		double speed = coords.get("r_shoulder_elev").getSpeedValue(si);

		cout << "Num States: " << numStates << " | Num Controls: " << numControls << " | Shoulder Speed: " << speed << std::endl;
		

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