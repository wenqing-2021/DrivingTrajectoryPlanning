#include "qp_solver/qp_solver.h"
#include <iostream>

int main() {
    planning::backend::QPSolver solver;
    solver.setVariableNums(1);
    solver.setConstraintNums(2);
    Eigen::SparseMatrix<double> h(1, 1), a(2, 1);
    h.insert(0, 0) = 2.0;
    a.insert(0, 0) = 1.0;
    a.insert(1, 0) = 1.0;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd lower(2), upper(2);
    lower << 1.0, -10.0;
    upper << 10.0, 2.0;
    if (!solver.Solve(h, g, a, lower, upper)) return 1;
    // Valid bound rows, but jointly infeasible: x >= 1 and x <= 0.
    upper(1) = 0.0;
    if (solver.Solve(h, g, a, lower, upper) || solver.GetResult()->size() != 0) {
        std::cerr << "An infeasible solve must not expose a solution or stale result\n";
        return 2;
    }
    return 0;
}
