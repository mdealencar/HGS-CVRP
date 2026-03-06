//
// Functional test using the C++ interfaces directly (Params, Genetic, Population).
// Converted from the original C test that used C_Interface.h.
//

#include "AlgorithmParameters.h"
#include "Genetic.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

struct TestSolution {
    double cost;
    double time;
    std::vector<std::vector<int>> routes;
};

// Flatten a vector-of-vectors distance matrix into a contiguous buffer
static std::vector<double> flatten(const std::vector<std::vector<double>>& mtx) {
    int n = static_cast<int>(mtx.size());
    std::vector<double> flat(n * n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            flat[i * n + j] = mtx[i][j];
    return flat;
}

// Run HGS and extract the best solution
static TestSolution solve(
    const std::vector<double>& x_coords,
    const std::vector<double>& y_coords,
    const std::vector<std::vector<double>>& dist_mtx,
    const std::vector<double>& service_time,
    const std::vector<double>& demands,
    double vehicleCapacity,
    double durationLimit,
    int nbVeh,
    bool isDurationConstraint,
    bool verbose,
    const AlgorithmParameters& ap,
    std::ostream& logStream)
{
    int nbNodes = static_cast<int>(demands.size());
    auto flat_dist = flatten(dist_mtx);
    const double* xp = x_coords.empty() ? nullptr : x_coords.data();
    const double* yp = y_coords.empty() ? nullptr : y_coords.data();
    Params params(xp, yp, flat_dist.data(), service_time.data(), demands.data(),
                  nbNodes, vehicleCapacity, durationLimit, nbVeh,
                  isDurationConstraint, verbose, ap, logStream);
    Genetic solver(params);
    solver.run();

    TestSolution result{};
    const Individual* best = solver.population.getBestFound();
    assert(best != nullptr);
    result.cost = best->eval.penalizedCost;
    result.time = params.elapsedSeconds();
    for (const auto& route : best->chromR) {
        if (!route.empty()) {
            result.routes.push_back(route);
        }
    }
    return result;
}

// Compute Euclidean distance matrix with integer rounding
static std::vector<std::vector<double>> euclidean_dist_mtx(
    const std::vector<double>& x, const std::vector<double>& y, bool rounding)
{
    int n = static_cast<int>(x.size());
    std::vector<std::vector<double>> mtx(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double dx = x[i] - x[j];
            double dy = y[i] - y[j];
            double dist = std::sqrt(dx * dx + dy * dy);
            if (rounding) dist = std::round(dist);
            mtx[i][j] = dist;
            mtx[j][i] = dist;
        }
    }
    return mtx;
}

static void print_solution(const TestSolution& sol) {
    std::cout << "Total Route Cost = " << sol.cost << std::endl;
    std::cout << "Total CPU Time = " << sol.time << std::endl;
    std::cout << "Number of Routes = " << sol.routes.size() << std::endl;
    for (size_t i = 0; i < sol.routes.size(); i++) {
        std::cout << "Route #" << i << ": [";
        for (size_t j = 0; j < sol.routes[i].size(); j++) {
            if (j > 0) std::cout << ", ";
            std::cout << sol.routes[i][j];
        }
        std::cout << "]" << std::endl;
    }
}

int main()
{
    AlgorithmParameters ap = default_algorithm_parameters();
    ap.timeLimit = 1.73;
    ap.nbIter = 10000;

    int n = 10;
    std::vector<double> x = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<double> y = {5, 4, 3, 2, 1, 9, 8, 7, 6, 5};
    std::vector<double> s(n, 0.0);
    std::vector<double> d = {0, 2, 3, 1, 2, 3, 1, 2, 3, 1};
    double v_cap = 10;
    double duration_limit = 100000000;
    bool isDurationConstraint = false;
    int max_nbVeh = 2;
    bool verbose = true;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #1 -----" << std::endl;
    // Solve from coordinates (with integer rounding)
    auto dist1 = euclidean_dist_mtx(x, y, true);
    auto sol = solve(x, y, dist1, s, d, v_cap, duration_limit, max_nbVeh,
                     isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol);
    assert(sol.cost == 29);
    assert(ap.useSwapStar == 1);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #2 -----" << std::endl;
    // Solve with asymmetric distance matrix
    std::vector<std::vector<double>> dist_mtx(n, std::vector<double>(n));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            dist_mtx[i][j] = std::sqrt((x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]));
            dist_mtx[i][j] *= (1 + 0.05 * i - 0.03 * j);
        }
    }

    auto sol2 = solve(x, y, dist_mtx, s, d, v_cap, duration_limit, max_nbVeh,
                      isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol2);
    assert(std::round(sol2.cost) == 32);
    assert(ap.useSwapStar == 1);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #3 -----" << std::endl;
    // Same dist_mtx but with zero coordinates (SWAP* deactivated)
    std::vector<double> zero(n, 0.0);
    auto sol3 = solve(zero, zero, dist_mtx, s, d, v_cap, duration_limit, max_nbVeh,
                      isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol3);
    assert(std::round(sol2.cost) == std::round(sol3.cost));
    assert(ap.useSwapStar == 1);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #4 -----" << std::endl;
    // Empty coordinates (SWAP* deactivated)
    std::vector<double> empty;
    auto sol4 = solve(empty, empty, dist_mtx, s, d, v_cap, duration_limit, max_nbVeh,
                      isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol4);
    assert(std::round(sol2.cost) == std::round(sol4.cost));
    assert(ap.useSwapStar == 1);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #5 -----" << std::endl;
    // Explicitly disable SWAP*
    ap.useSwapStar = 0;
    auto sol5 = solve(empty, empty, dist_mtx, s, d, v_cap, duration_limit, max_nbVeh,
                      isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol5);
    assert(std::round(sol2.cost) == std::round(sol5.cost));
    assert(ap.useSwapStar == 0);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #6 (redundant duration constraint) -----" << std::endl;
    ap.useSwapStar = 0;
    duration_limit = 1000;
    isDurationConstraint = true;
    auto sol6 = solve(empty, empty, dist_mtx, s, d, v_cap, duration_limit, max_nbVeh,
                      isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol6);
    assert(std::round(sol5.cost) == std::round(sol6.cost));

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #7 (tight duration constraint) -----" << std::endl;
    // Build rounded Euclidean distance matrix
    std::vector<std::vector<double>> rounded_dist_mtx(n, std::vector<double>(n));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            rounded_dist_mtx[i][j] = std::round(
                std::sqrt((x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j])));
            std::cout << static_cast<int>(rounded_dist_mtx[i][j]) << " ";
        }
        std::cout << std::endl;
    }

    ap.useSwapStar = 1;
    ap.seed = 12;
    max_nbVeh = 5;
    duration_limit = 18;
    isDurationConstraint = true;
    auto sol7 = solve(empty, empty, rounded_dist_mtx, s, d, v_cap, duration_limit, max_nbVeh,
                      isDurationConstraint, verbose, ap, std::cout);
    print_solution(sol7);

    // Verify each route respects the duration limit
    for (size_t r = 0; r < sol7.routes.size(); r++) {
        const auto& route = sol7.routes[r];
        double duration = 0.0;
        // depot -> first
        duration += rounded_dist_mtx[0][route[0]];
        for (size_t k = 0; k + 1 < route.size(); k++) {
            duration += rounded_dist_mtx[route[k]][route[k + 1]];
        }
        // last -> depot
        duration += rounded_dist_mtx[route.back()][0];
        assert(duration <= duration_limit);
        std::cout << "route #" << r << " duration = " << duration << std::endl;
    }
    std::cout << "duration_limit = " << duration_limit << std::endl;
    assert(sol7.cost == 42);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "-------- test.cpp #8 (log capture) -----" << std::endl;
    // Verify per-instance log stream works (log goes to stringstream, not stdout)
    std::ostringstream log_capture;
    ap = default_algorithm_parameters();
    ap.timeLimit = 0.5;
    auto dist8 = euclidean_dist_mtx(x, y, true);
    auto sol8 = solve(x, y, dist8, s, d, v_cap, 100000000, 2,
                      false, true, ap, log_capture);
    std::string log = log_capture.str();
    assert(!log.empty());
    assert(log.find("GENETIC ALGORITHM") != std::string::npos);
    std::cout << "Log captured: " << log.size() << " bytes" << std::endl;
    assert(sol8.cost == 29);

    std::cout << "======== ALL TESTS PASSED ========" << std::endl;
    return 0;
}
