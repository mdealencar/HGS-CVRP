#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
#include "InstanceCVRPLIB.h"
#include <stdexcept>
using namespace std;

// Exit status codes. Kept as explicit values rather than EXIT_SUCCESS/EXIT_FAILURE so that
// the contract documented in the README holds on every platform.
static constexpr int EXIT_OK    = 0;	// A solution was found and written
static constexpr int EXIT_ERROR = 1;	// The run failed: unreadable instance, invalid data, no solution
static constexpr int EXIT_USAGE = 2;	// The command line itself was malformed

int main(int argc, char *argv[])
{
	try
	{
		// Reading the arguments of the program
		CommandLine commandline(argc, argv);

		// Print all algorithm parameter values
		if (commandline.verbose) print_algorithm_parameters(commandline.ap, std::cout);

		// Reading the data file and initializing some data structures
		if (commandline.verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		InstanceCVRPLIB cvrp(commandline.pathInstance, commandline.isRoundingInteger);

		// Flatten the distance matrix for the new Params interface
		int nbNodes = (int)cvrp.demands.size();
		std::vector<double> flat_dist(nbNodes * nbNodes);
		for (int i = 0; i < nbNodes; i++)
			for (int j = 0; j < nbNodes; j++)
				flat_dist[i * nbNodes + j] = cvrp.dist_mtx[i][j];

		Params params(cvrp.x_coords.data(), cvrp.y_coords.data(), flat_dist.data(),
			          cvrp.service_time.data(), cvrp.demands.data(), nbNodes,
			          cvrp.vehicleCapacity, cvrp.durationLimit, commandline.nbVeh,
			          cvrp.isDurationConstraint, commandline.verbose, commandline.ap);

		// Running HGS
		Genetic solver(params);
		solver.run();
		
		// Exporting the best solution
		if (solver.population.getBestFound() == NULL)
			throw std::runtime_error("No feasible solution was found, nothing to write.");

		if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
		solver.population.exportCVRPLibFormat(*solver.population.getBestFound(),commandline.pathSolution);
		solver.population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance);
	}
	catch (const CommandLineError& e) { std::cerr << "EXCEPTION | " << e.what() << std::endl; return EXIT_USAGE; }
	catch (const std::exception& e) { std::cerr << "EXCEPTION | " << e.what() << std::endl; return EXIT_ERROR; }
	return EXIT_OK;
}
