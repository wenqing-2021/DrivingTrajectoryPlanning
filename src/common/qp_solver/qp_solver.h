#include "OsqpEigen/OsqpEigen.h"

namespace planning {
namespace backend {

class QPSolver {
  public:
    QPSolver(bool verbose = false) {
        // init solver
        solver_ = OsqpEigen::Solver();
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setVerbosity(verbose);
    };
    ~QPSolver() = default;

    bool Solve(const Eigen::SparseMatrix<double>& H, Eigen::VectorXd& g, const Eigen::SparseMatrix<double>& A,
               Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound) {
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

        // solve the problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        // store the result
        result_ = solver_.getSolution();
        return true;
    };

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
    OsqpEigen::Solver solver_;
    Eigen::VectorXd   result_;
};


}   // namespace backend
}   // namespace planning