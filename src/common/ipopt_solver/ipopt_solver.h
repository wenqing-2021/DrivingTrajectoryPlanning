
#pragma once
#include <Eigen/Core>
#include <cppad/ipopt/solve.hpp>

namespace planning {
namespace backend {

class FG_eval {
  public:
    typedef CppAD::vector<CppAD::AD<double>> ADvector;
    FG_eval()  = default;
    ~FG_eval() = default;
    void updateProblemSize(const std::size_t n_vars, const std::size_t n_cons) {
        n_vars_ = n_vars;
        n_cons_ = n_cons;
    }

    void operator()(ADvector& fg, const ADvector& x) {
        // set objective and constraints function
        // f(x) objective function
        fg[0] = getCostFunction(x);
        // constraints
        const auto constraint_func = getConstraints(x);
        for (std::size_t i = 0; i < n_cons_; ++i) { fg[i + 1] = constraint_func[i]; }
        return;
    }

    virtual CppAD::AD<double> getCostFunction(const ADvector& x) = 0;
    virtual ADvector          getConstraints(const ADvector& x)  = 0;

  private:
    std::size_t n_vars_;   // number of variables
    std::size_t n_cons_;   // number of constraints
};

class IpoptSolver {
  public:
    typedef CppAD::vector<double> Dvector;
    IpoptSolver(std::string options)
        : options_(options) {}
    ~IpoptSolver() = default;

    bool solve(const Eigen::VectorXd& x0, const Eigen::VectorXd& xl, const Eigen::VectorXd& xu,
               const Eigen::VectorXd& gl, const Eigen::VectorXd& gu, FG_eval* fg_eval) {
        // 1. get the objective function
        setInitialValue(x0);

        return true;
    }

  private:
    Dvector                             x0_;   // initial value of variables
    CppAD::ipopt::solve_result<Dvector> solution_;
    std::string                         options_;

    bool setOptions() { return true; }
    void setInitialValue(const Eigen::VectorXd& x0) {
        x0_.resize(x0.size());
        for (std::size_t i = 0; i < x0.size(); ++i) { x0_[i] = x0[i]; }
    }
    bool setObjective() { return true; }
    bool setConstraints() { return true; }

  private:
};
}   // namespace backend
}   // namespace planning