#include "Params.h"

Params::Params(
	const double* x_coords,
	const double* y_coords,
	const double* dist_mtx,
	const double* service_time,
	const double* demands,
	int nbNodes,
	double vehicleCapacity,
	double durationLimit,
	int nbVeh,
	bool isDurationConstraint,
	bool verbose,
	const AlgorithmParameters& ap,
	std::ostream& logStream
)
	: ap(ap), isDurationConstraint(isDurationConstraint), nbVehicles(nbVeh), durationLimit(durationLimit),
	  vehicleCapacity(vehicleCapacity), timeCost_(dist_mtx), timeCostN_(nbNodes), verbose(verbose), logStream(logStream)
{
	// This marks the starting time of the algorithm
	startTime = ThreadCpuTimer::now();

	nbClients = nbNodes - 1; // Need to substract the depot from the number of nodes
	totalDemand = 0.;
	maxDemand = 0.;

	// Initialize RNG
	ran.seed(ap.seed);

	// Coordinates are considered provided when both pointers are non-null.
	// In that case, callers must provide at least nbNodes values in each array.
	areCoordinatesProvided = (x_coords != nullptr && y_coords != nullptr);

	cli = std::vector<Client>(nbClients + 1);
	for (int i = 0; i <= nbClients; i++)
	{
		// To disable coordinates/SWAP* in this API, pass nullptr for both coordinate pointers.
		if (ap.useSwapStar == 1 && areCoordinatesProvided)
		{
			cli[i].coordX = x_coords[i];
			cli[i].coordY = y_coords[i];
			cli[i].polarAngle = CircleSector::positive_mod(
				32768. * atan2(cli[i].coordY - cli[0].coordY, cli[i].coordX - cli[0].coordX) / PI);
		}
		else
		{
			cli[i].coordX = 0.0;
			cli[i].coordY = 0.0;
			cli[i].polarAngle = 0.0;
		}

		cli[i].serviceDuration = service_time[i];
		cli[i].demand = demands[i];
		if (cli[i].demand > maxDemand) maxDemand = cli[i].demand;
		totalDemand += cli[i].demand;
	}

	if (verbose && ap.useSwapStar == 1 && !areCoordinatesProvided)
		logStream << "----- NO COORDINATES HAVE BEEN PROVIDED, SWAP* NEIGHBORHOOD WILL BE DEACTIVATED BY DEFAULT" << std::endl;

	// Default initialization if the number of vehicles has not been provided by the user
	if (nbVehicles < 0)
	{
		nbVehicles = (int)std::ceil(1.3*totalDemand/vehicleCapacity) + 3;  // Safety margin: 30% + 3 more vehicles than the trivial bin packing LB
		if (verbose)
			logStream << "----- FLEET SIZE WAS NOT SPECIFIED: DEFAULT INITIALIZATION TO " << nbVehicles << " VEHICLES" << std::endl;
	}
	else
	{
		if (verbose)
			logStream << "----- FLEET SIZE SPECIFIED: SET TO " << nbVehicles << " VEHICLES" << std::endl;
	}

	// Calculation of the maximum distance
	maxDist = 0.;
	for (int i = 0; i <= nbClients; i++)
		for (int j = 0; j <= nbClients; j++)
			if (timeCost(i, j) > maxDist) maxDist = timeCost(i, j);

	// Calculation of the correlated vertices for each customer (for the granular restriction)
	correlatedVertices = std::vector<std::vector<int> >(nbClients + 1);
	std::vector<std::set<int> > setCorrelatedVertices = std::vector<std::set<int> >(nbClients + 1);
	std::vector<std::pair<double, int> > orderProximity;
	for (int i = 1; i <= nbClients; i++)
	{
		orderProximity.clear();
		for (int j = 1; j <= nbClients; j++)
			if (i != j) orderProximity.emplace_back(timeCost(i, j), j);
		std::sort(orderProximity.begin(), orderProximity.end());

		for (int j = 0; j < std::min<int>(ap.nbGranular, nbClients - 1); j++)
		{
			// If i is correlated with j, then j should be correlated with i
			setCorrelatedVertices[i].insert(orderProximity[j].second);
			setCorrelatedVertices[orderProximity[j].second].insert(i);
		}
	}

	// Filling the vector of correlated vertices
	for (int i = 1; i <= nbClients; i++)
		for (int x : setCorrelatedVertices[i])
			correlatedVertices[i].push_back(x);

	// Safeguards to avoid possible numerical instability in case of instances containing arbitrarily small or large numerical values
	if (maxDist < 0.1 || maxDist > 100000)
		throw std::string(
			"The distances are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (maxDemand < 0.1 || maxDemand > 100000)
		throw std::string(
			"The demand quantities are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (nbVehicles < std::ceil(totalDemand / vehicleCapacity))
		throw std::string("Fleet size is insufficient to service the considered clients.");

	// A reasonable scale for the initial values of the penalties
	penaltyDuration = 1;
	penaltyCapacity = std::max<double>(0.1, std::min<double>(1000., maxDist / maxDemand));

	if (verbose)
		logStream << "----- INSTANCE SUCCESSFULLY LOADED WITH " << nbClients << " CLIENTS AND " << nbVehicles << " VEHICLES" << std::endl;
}

double Params::elapsedSeconds() const
{
	return ThreadCpuTimer::elapsedSeconds(startTime);
}
