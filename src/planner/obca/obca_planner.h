#pragma once
#include "logger.h"
#include "map.h"
#include "math/polygon2d.h"
#include "vehicle_model/kinematic_model.h"
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
    OBCASolver(const kinematic_model::VehicleParam& vehicle_param, const std::shared_ptr<map::Map>& map_ptr = nullptr)
        : map_ptr_(map_ptr) {
        initializeParameters(vehicle_param);
    };
    ~OBCASolver() = default;
    bool Solve(const vehicle_model::sdv_traj& init_trajectory, const vehicle_model::VehiclePose& start_pose,
               const vehicle_model::VehiclePose& goal_pose);

    inline const std::vector<Eigen::Vector4d>& GetStatesResult() const { return states_result_; }
    inline const std::vector<Eigen::Vector2d>& GetControlsResult() const { return controls_result_; }

    bool getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd* obstacle_A, Eigen::VectorXd* obstacle_b,
                      std::size_t i);
    bool getRotationMatrix(const double theta, Eigen::Matrix2d* R);

    bool getMovementMatrix(const double x, const double y, const double theta, Eigen::Matrix<double, 2, 1>* t);

  private:
    bool setInitVariable();
    bool initializeParameters(
        const kinematic_model::VehicleParam& vehicle_param);   // Initialize the parameters for the OBCA algorithm
    bool buildInequalConstraint();
    bool buildEqualConstraint();
    bool buildCostFunction(const vehicle_model::sdv_path& init_trajectory);
    bool buildLowUpperBound();

    Eigen::Matrix<double, 4, 2> G_;   // ego vehicle matrix: Gx <= g
    Eigen::Matrix<double, 4, 1> g_;   // Control input matrix

    double                  offset_;   // the distance from the rear axle to the vehicle center
    constexpr static double kEpsilon = 1e-5;
    std::size_t             N_;             // Number of discretized steps
    std::size_t             state_num_;     // Number of states
    std::size_t             control_num_;   // Number of controls

    std::vector<Eigen::Vector4d> states_result_;     // Resulting states after optimization
    std::vector<Eigen::Vector2d> controls_result_;   // Resulting controls after optimization

    std::shared_ptr<map::Map> map_ptr_;
};
}   // namespace backend
}   // namespace planning
