
#include <Eigen/Core>
#include <vector>

namespace planning {
namespace backend {
class OBCASolver {
    /*
    Solve the optimization problem using the Optimization-Based Collision Avoidance.
    The control u = [\sigma, a] is the control input, where \sigma is the steering angle and a is the acceleration.
    The state x = [x, y, \theta, v] is the state of the vehicle, where x and y are the position coordinates,
    \theta is the heading angle, and v is the velocity.
    */
  public:
    OBCASolver();
    ~OBCASolver() = default;
    bool Solve();

    inline const std::vector<Eigen::Vector4d>& GetStatesResult() const { return states_result_; }
    inline const std::vector<Eigen::Vector2d>& GetControlsResult() const { return controls_result_; }

  private:
    bool initializeParameters();   // Initialize the parameters for the OBCA algorithm
    bool solveLambdaMu();
    bool solveVelAcc();
    bool solveTime();
    bool solveStates();

    std::vector<Eigen::Vector4d> states_result_;     // Resulting states after optimization
    std::vector<Eigen::Vector2d> controls_result_;   // Resulting controls after optimization

    // Placeholder for the OBCA algorithm parameters and variables
    // This should include the necessary matrices, vectors, and parameters for the OBCA algorithm
};
}   // namespace backend
}   // namespace planning
