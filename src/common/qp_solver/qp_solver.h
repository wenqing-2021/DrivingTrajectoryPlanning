#pragma once

#include "OsqpEigen/OsqpEigen.h"
#include <iostream>

namespace planning {
namespace backend {

class QPSolver {
  public:
    QPSolver(bool verbose = false, int max_iterations = 4000, bool reuse = false)
        : reuse_(reuse) {
        // init solver
        solver_ = OsqpEigen::Solver();
        solver_.settings()->setWarmStart(!reuse);
        solver_.settings()->setVerbosity(verbose);
        solver_.settings()->setMaxIteration(max_iterations);
    };
    ~QPSolver() = default;

    bool Solve(const Eigen::SparseMatrix<double>& H, Eigen::VectorXd& g, const Eigen::SparseMatrix<double>& A,
               Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound) {
        result_.resize(0);
        if (reuse_ && initialized_) {
            if (!solver_.updateHessianMatrix(H) || !solver_.updateLinearConstraintsMatrix(A) ||
                !solver_.updateGradient(g) || !solver_.updateBounds(lowerBound, upperBound))
                return false;
        } else {
            // clear matrix
            solver_.data()->clearHessianMatrix();
            solver_.data()->clearLinearConstraintsMatrix();
            solver_.clearSolver();
            if (!solver_.data()->setHessianMatrix(H)) return false;
            if (!solver_.data()->setGradient(g)) return false;
            if (!solver_.data()->setLinearConstraintsMatrix(A)) return false;
            if (!solver_.data()->setLowerBound(lowerBound)) return false;
            if (!solver_.data()->setUpperBound(upperBound)) return false;

            // instantiate the solver
            if (!solver_.initSolver()) return false;

            initialized_ = true;
        }

        // solve the problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        const auto status = solver_.getStatus();
        if (status != OsqpEigen::Status::Solved && status != OsqpEigen::Status::SolvedInaccurate) {
            const auto* info = solver_.solver()->info;
            std::cerr << "OSQP failed: " << info->status << " (iterations=" << info->iter
                      << ", primal_residual=" << info->prim_res
                      << ", dual_residual=" << info->dual_res << ")\n";
            return false;
        }
        // store the result
        result_ = solver_.getSolution();
        return true;
    };

    // Keep matrix buffers owned by the caller, but do not carry adaptive rho /
    // scaling from a previous planning request into a new initial trajectory.
    void Reset() {
        solver_.clearSolver();
        initialized_ = false;
        result_.resize(0);
    }

    void setVariableNums(int N) {
        // set the number of variables
        solver_.data()->setNumberOfVariables(N);
    };

    void setConstraintNums(int M) {
        // set the number of constraints
        solver_.data()->setNumberOfConstraints(M);
    };

    const Eigen::VectorXd* const GetResult() { return &result_; };

  private:
    bool              reuse_       = false;
    bool              initialized_ = false;
    OsqpEigen::Solver solver_;
    Eigen::VectorXd   result_;
};


}   // namespace backend
}   // namespace planning