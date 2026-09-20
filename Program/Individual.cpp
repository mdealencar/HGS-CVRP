#include "Individual.h"
#include <stdexcept>

void Individual::evaluateCompleteCost(const Params & params)
{
	eval = EvalIndiv();
	for (int r = 0; r < params.nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			double distance = params.timeCost(0, chromR[r][0]);
			double load = params.cli[chromR[r][0]].demand;
			double service = params.cli[chromR[r][0]].serviceDuration;
			predecessors[chromR[r][0]] = 0;
			for (int i = 1; i < (int)chromR[r].size(); i++)
			{
				distance += params.timeCost(chromR[r][i-1], chromR[r][i]);
				load += params.cli[chromR[r][i]].demand;
				service += params.cli[chromR[r][i]].serviceDuration;
				predecessors[chromR[r][i]] = chromR[r][i-1];
				successors[chromR[r][i-1]] = chromR[r][i];
			}
			successors[chromR[r][chromR[r].size()-1]] = 0;
			distance += params.timeCost(chromR[r][chromR[r].size()-1], 0);
			eval.distance += distance;
			eval.nbRoutes++;
			if (load > params.vehicleCapacity) eval.capacityExcess += load - params.vehicleCapacity;
			if (distance + service > params.durationLimit) eval.durationExcess += distance + service - params.durationLimit;
		}
	}

	eval.penalizedCost = eval.distance + eval.capacityExcess*params.penaltyCapacity + eval.durationExcess*params.penaltyDuration;
	eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);
}

void Individual::allocateChromosomes(const Params & params)
{
	successors = std::vector <int>(params.nbClients + 1);
	predecessors = std::vector <int>(params.nbClients + 1);
	chromR = std::vector < std::vector <int> >(params.nbVehicles);
	chromT.clear();
	chromT.reserve(params.nbClients);
	eval.penalizedCost = 1.e30;
}

Individual::Individual(Params & params, const std::vector < std::vector <int> > & routes)
{
	allocateChromosomes(params);

	// chromR has one entry per vehicle, so a solution using more routes than that would write past its end
	if ((int)routes.size() > params.nbVehicles)
		throw std::invalid_argument("Input solution uses " + std::to_string(routes.size())
			+ " routes, more than the fleet size of " + std::to_string(params.nbVehicles));

	// The chromosomes, and the crossover operating on them, are only meaningful if every client is
	// visited exactly once, so a malformed solution is rejected rather than silently corrupting them
	std::vector <bool> isVisited(params.nbClients + 1, false);
	for (int r = 0; r < (int)routes.size(); r++)
	{
		for (int client : routes[r])
		{
			if (client < 1 || client > params.nbClients)
				throw std::invalid_argument("Input solution visits " + std::to_string(client)
					+ ", which is not a client index in [1, " + std::to_string(params.nbClients) + "]");
			if (isVisited[client])
				throw std::invalid_argument("Input solution visits client " + std::to_string(client) + " more than once");
			isVisited[client] = true;
			chromT.push_back(client);
			chromR[r].push_back(client);
		}
	}
	if ((int)chromT.size() != params.nbClients)
	{
		// Without duplicates or out-of-range entries, too few visits means a client was left out
		int missing = 1;
		while (missing <= params.nbClients && isVisited[missing]) missing++;
		throw std::invalid_argument("Input solution does not visit client " + std::to_string(missing));
	}

	evaluateCompleteCost(params);
}

Individual::Individual(Params & params)
{
	allocateChromosomes(params);
	for (int i = 1; i <= params.nbClients; i++) chromT.push_back(i);
	std::shuffle(chromT.begin(), chromT.end(), params.ran);
}

Individual Individual::fromCVRPLibFile(Params & params, const std::string & fileName)
{
	std::ifstream inputFile(fileName);
	if (!inputFile.is_open())
		throw std::runtime_error("Impossible to open solution file provided in input in : " + fileName);

	double readCost;
	std::vector < std::vector <int> > routes;
	std::string inputString;
	inputFile >> inputString;
	// Loops in the input file as long as the first line keyword is "Route"
	while (inputString == "Route")
	{
		inputFile >> inputString;
		getline(inputFile, inputString);
		std::stringstream ss(inputString);
		routes.emplace_back();
		int inputCustomer;
		while (ss >> inputCustomer) // Loops as long as there is an integer to read in this route
			routes.back().push_back(inputCustomer);
		inputFile >> inputString;
	}
	if (inputString == "Cost") inputFile >> readCost;
	else throw std::runtime_error("Unexpected token in input solution");

	// Validates the routes and evaluates the solution
	Individual indiv(params, routes);

	// Some safety checks and printouts, specific to reading a solution announced by a file
	if (!indiv.eval.isFeasible) throw std::invalid_argument("Input solution is infeasible");
	if (indiv.eval.penalizedCost != readCost) throw std::invalid_argument("Input solution has a different cost than announced in the file");
	if (params.verbose) params.logStream << "----- INPUT SOLUTION HAS BEEN SUCCESSFULLY READ WITH COST " << indiv.eval.penalizedCost << std::endl;
	return indiv;
}
