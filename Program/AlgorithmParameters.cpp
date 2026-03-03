//
#include "Logger.h"
// Created by chkwon on 3/23/22.
//

#include "AlgorithmParameters.h"
#include <iostream>

extern "C"
struct AlgorithmParameters default_algorithm_parameters() {
	struct AlgorithmParameters ap{};

	ap.nbGranular = 20;
	ap.mu = 25;
	ap.lambda = 40;
	ap.nbElite = 4;
	ap.nbClose = 5;

	ap.nbIterPenaltyManagement = 100;
	ap.targetFeasible = 0.2;
	ap.penaltyDecrease = 0.85;
	ap.penaltyIncrease = 1.2;

	ap.seed = 0;
	ap.nbIter = 20000;
	ap.nbIterTraces = 500;
	ap.timeLimit = 0;
	ap.useSwapStar = 1;

	return ap;
}

void print_algorithm_parameters(const AlgorithmParameters & ap)
{
	hgs_log_stream() << "=========== Algorithm Parameters =================" << std::endl;
	hgs_log_stream() << "---- nbGranular              is set to " << ap.nbGranular << std::endl;
	hgs_log_stream() << "---- mu                      is set to " << ap.mu << std::endl;
	hgs_log_stream() << "---- lambda                  is set to " << ap.lambda << std::endl;
	hgs_log_stream() << "---- nbElite                 is set to " << ap.nbElite << std::endl;
	hgs_log_stream() << "---- nbClose                 is set to " << ap.nbClose << std::endl;
	hgs_log_stream() << "---- nbIterPenaltyManagement is set to " << ap.nbIterPenaltyManagement << std::endl;
	hgs_log_stream() << "---- targetFeasible          is set to " << ap.targetFeasible << std::endl;
	hgs_log_stream() << "---- penaltyDecrease         is set to " << ap.penaltyDecrease << std::endl;
	hgs_log_stream() << "---- penaltyIncrease         is set to " << ap.penaltyIncrease << std::endl;
	hgs_log_stream() << "---- seed                    is set to " << ap.seed << std::endl;
	hgs_log_stream() << "---- nbIter                  is set to " << ap.nbIter << std::endl;
	hgs_log_stream() << "---- nbIterTraces            is set to " << ap.nbIterTraces << std::endl;
	hgs_log_stream() << "---- timeLimit               is set to " << ap.timeLimit << std::endl;
	hgs_log_stream() << "---- useSwapStar             is set to " << ap.useSwapStar << std::endl;
	hgs_log_stream() << "==================================================" << std::endl;
}
