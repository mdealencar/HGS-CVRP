//
// Created by chkwon on 3/23/22.
//

#include "AlgorithmParameters.h"
#include <iostream>

AlgorithmParameters default_algorithm_parameters() {
	AlgorithmParameters ap{};

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

void print_algorithm_parameters(const AlgorithmParameters & ap, std::ostream& out)
{
	out << "=========== Algorithm Parameters =================" << std::endl;
	out << "---- nbGranular              is set to " << ap.nbGranular << std::endl;
	out << "---- mu                      is set to " << ap.mu << std::endl;
	out << "---- lambda                  is set to " << ap.lambda << std::endl;
	out << "---- nbElite                 is set to " << ap.nbElite << std::endl;
	out << "---- nbClose                 is set to " << ap.nbClose << std::endl;
	out << "---- nbIterPenaltyManagement is set to " << ap.nbIterPenaltyManagement << std::endl;
	out << "---- targetFeasible          is set to " << ap.targetFeasible << std::endl;
	out << "---- penaltyDecrease         is set to " << ap.penaltyDecrease << std::endl;
	out << "---- penaltyIncrease         is set to " << ap.penaltyIncrease << std::endl;
	out << "---- seed                    is set to " << ap.seed << std::endl;
	out << "---- nbIter                  is set to " << ap.nbIter << std::endl;
	out << "---- nbIterTraces            is set to " << ap.nbIterTraces << std::endl;
	out << "---- timeLimit               is set to " << ap.timeLimit << std::endl;
	out << "---- useSwapStar             is set to " << ap.useSwapStar << std::endl;
	out << "==================================================" << std::endl;
}
