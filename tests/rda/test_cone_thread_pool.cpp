#include "planner/rda/rda_workspace.h"
#include <Eigen/Core>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace planning::backend;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// min t with ||(x, y)|| <= t: an independent SOCP per obstacle.
std::vector<ConeWorkspace> makeProblems(int count, bool extra_cone) {
    std::vector<ConeWorkspace> problems(count);
    for (int i = 0; i < count; ++i) {
        auto&     problem = problems[i];
        const int shift   = extra_cone ? 1 : 0;
        problem.coefficients.begin(extra_cone ? 5 : 4, 3);
        problem.coefficients.emplace_back(0, 0, -1);
        if (extra_cone) problem.coefficients.emplace_back(1, 1, -1);
        problem.coefficients.emplace_back(1 + shift, 2, -1);
        problem.coefficients.emplace_back(2 + shift, 0, -1);
        problem.coefficients.emplace_back(3 + shift, 1, -1);
        problem.coefficients.finish();
        problem.c    = Eigen::Vector3d(0, 0, 1);
        problem.h    = Eigen::VectorXd::Zero(extra_cone ? 5 : 4);
        problem.h(0) = -1.0 - 0.25 * i;
        problem.equality.resize(0, 3);
        problem.b.resize(0);
        problem.cones = Eigen::VectorXi::Constant(1, 3);
    }
    return problems;
}

int main() {
    try {
        ConeThreadPool             pool;
        std::vector<ConeWorkspace> problems;
        for (int shape = 0; shape < 2; ++shape) {
            pool.reset();
            problems = makeProblems(5 + 2 * shape, shape == 1);
            std::vector<Eigen::VectorXd> expected;
            for (auto& problem : problems) {
                problem.solve();
                expected.push_back(problem.result);
            }
            // Reuse across worker counts and rounds; results must not depend on either.
            for (int workers : {1, 3, 4, 64}) {
                for (int round = 0; round < 2; ++round) {
                    require(pool.solve(problems, workers, 5000), "Threaded SOCP round failed");
                    for (std::size_t i = 0; i < problems.size(); ++i) {
                        require((problems[i].result - expected[i]).norm() < 1e-10, "Threaded SOCP differs from serial");
                    }
                }
            }
        }
        require(!pool.solve(problems, 0, 1000), "Zero workers must be rejected");
        require(!pool.solve(problems, 2, 0), "Nonpositive timeout must be rejected");

        // A changed obstacle count must be recoverable through reset().
        pool.reset();
        auto resized = makeProblems(3, false);
        require(pool.solve(resized, 2, 5000), "Pool reuse after reset failed");
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
