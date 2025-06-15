#include <Eigen/Core>

namespace planning {
namespace backend {
class ADMMSolver {
  public:
    ADMMSolver();
    ~ADMMSolver() = default;
    bool Solve();


    inline const std::vector<Eigen::Vector4d>& GetStatesResult() const { return states_result_; }
    inline const std::vector<Eigen::Vector2d>& GetControlsResult() const { return controls_result_; }

  private:
    std::vector<Eigen::Vector4d> states_result_;     // Resulting states after optimization
    std::vector<Eigen::Vector2d> controls_result_;   // Resulting controls after optimization

    // Placeholder for the ADMM algorithm parameters and variables
    // This should include the necessary matrices, vectors, and parameters for the ADMM algorithm
    // For example:
    // Eigen::MatrixXd hessian_matrix_;
    // Eigen::VectorXd gradient_;
    // Eigen::VectorXd lower_bound_;
    // Eigen::VectorXd upper_bound_;
    // etc.
};
}   // namespace backend
}   // namespace planning
