#include "planner/rda/rda_workspace.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace planning::backend;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
int main() {
    try {
        SparseValues values;
        for (int pass = 0; pass < 4; ++pass) {
            values.begin(2, 2);
            values.emplace_back(1, 0, pass);
            values.emplace_back(0, 1, 2);
            values.emplace_back(1, 0, -0.5 * pass);
            const auto& m = values.finish();
            require(m.nonZeros() == 2 && m.coeff(1, 0) == 0.5 * pass, "Sparse zeros/duplicates changed");
        }
        values.begin(3, 3);
        values.emplace_back(2, 2, 1);
        require(values.finish().coeff(2, 2) == 1, "Dimension change failed");

        QPSolver qp(false, 4000, true);
        qp.setVariableNums(1);
        qp.setConstraintNums(1);
        Eigen::SparseMatrix<double> H(1, 1), C(1, 1);
        H.insert(0, 0) = 2;
        C.insert(0, 0) = 1;
        Eigen::VectorXd g(1), lo(1), hi(1);
        g << -2;
        lo << 0;
        hi << 10;
        require(qp.Solve(H, g, C, lo, hi), "Initial QP failed");
        g << -4;
        require(qp.Solve(H, g, C, lo, hi) && std::abs((*qp.GetResult())(0) - 2) < 0.01, "QP update ignored");
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
