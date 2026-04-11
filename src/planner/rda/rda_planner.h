#pragma once

#include "eicos.hpp"
#include "logger.h"
#include "map.h"
#include "params.pb.h"
#include "vehicle_model/kinematic_model.h"
#include <Eigen/Core>
#include <memory>
#include <vector>

namespace planning {
namespace backend {

struct RDAOptimizationResult {
    std::vector<Eigen::Vector4d> states;     // [x, y, theta, v]
    std::vector<Eigen::Vector2d> controls;   // [acceleration, steering_angle]
    double                       cost;
    int                          iterations;
    bool                         is_feasible;
};

class RDASolver {
  public:
    // Constructor and Destructor
    RDASolver(const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
              const std::shared_ptr<map::Map>& map_ptr, const params::RDAParams& rda_params, double dt = 0.1);
    ~RDASolver() = default;

    // Main solving interface
    bool Process(const vehicle_model::sdv_path& init_path, std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                 std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    // Result accessors
    const RDAOptimizationResult&        GetOptimizationResult() const;
    const std::vector<Eigen::Vector4d>& GetStatesResult() const;
    const std::vector<Eigen::Vector2d>& GetControlsResult() const;
    const std::vector<Eigen::Vector4d>& GetInitialStates() const;
    double                              GetOptimalCost() const;

    // Utility methods
    static vehicle_model::sdv_path      Vec3dToSdvPath(const std::vector<Eigen::Vector3d>& path);
    static std::vector<Eigen::Vector3d> SdvPathToVec3d(const vehicle_model::sdv_path& path);

  private:
    // SOCP problem formulation
    bool formSOCPProblem(const vehicle_model::sdv_path&               init_path,
                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    // Initialize optimization variables
    bool initializeOptimizationVariables(const vehicle_model::sdv_path&               init_path,
                                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    // Set constraint bounds
    bool setVariableBounds(Eigen::VectorXd& x_lower, Eigen::VectorXd& x_upper);

    bool setDynamicConstraints(Eigen::MatrixXd& A_constraint, Eigen::VectorXd& b_constraint);

    bool setCollisionAvoidanceConstraints(Eigen::MatrixXd& A_collision, std::vector<Eigen::VectorXd>& b_collision_vec);

    bool setControlConstraints(Eigen::MatrixXd& A_control, Eigen::VectorXd& b_control);

    // Solve SOCP using ECOS or similar solver
    bool solveSOCPProblem(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& A,
                          const Eigen::VectorXd& b, Eigen::VectorXd& solution);

    // Extract and process results
    bool extractOptimizationResult(const Eigen::VectorXd& solution);

    // RDA-specific iterative refinement
    bool performRDAIteration(Eigen::VectorXd& solution, int max_iterations);

    bool checkSolutionFeasibility(const Eigen::VectorXd& solution);

    // Obstacle and constraint processing
    bool getObstacleSOCPForm(const std::shared_ptr<map::Map>& map_ptr, std::vector<Eigen::MatrixXd>& A_obstacles,
                             std::vector<Eigen::VectorXd>& b_obstacles);

    // Cost function setup
    bool setupQuadraticCostFunction(Eigen::MatrixXd& P, Eigen::VectorXd& q);

    // Vehicle geometry and kinematics
    bool getVehicleGeometryMatrix(Eigen::MatrixXd& G, Eigen::VectorXd& g);

    bool discreteKinematicConstraint(double x_prev, double y_prev, double theta_prev, double v_prev,
                                     double acceleration, double steering_angle, double& x_next, double& y_next,
                                     double& theta_next, double& v_next);

    // Member variables
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
    std::shared_ptr<map::Map>                      map_ptr_;
    params::RDAParams                              rda_params_;

    double dt_;              // Time discretization step
    size_t N_;               // Number of time steps
    size_t state_dim_;       // State dimension (4: x, y, theta, v)
    size_t control_dim_;     // Control dimension (2: acceleration, steering_angle)
    size_t total_var_dim_;   // Total optimization variable dimension

    // Optimization problem data
    Eigen::VectorXd       initial_variables_;
    Eigen::VectorXd       optimal_solution_;
    RDAOptimizationResult optimization_result_;

    // Results storage
    std::vector<Eigen::Vector4d> states_result_;
    std::vector<Eigen::Vector2d> controls_result_;
    std::vector<Eigen::Vector4d> initial_states_;

    // Algorithm parameters
    double              convergence_tolerance_;
    int                 max_rda_iterations_;
    double              penalty_weight_;
    bool                use_warm_start_;
    std::vector<double> cost_history_;   // For convergence monitoring

    // Vehicle constraints
    double max_velocity_;
    double max_acceleration_;
    double max_steering_angle_;
    double wheel_base_;
    double vehicle_length_;
    double vehicle_width_;

    // Safety and numerical parameters
    constexpr static double kSafetyMargin     = 0.3;   // m
    constexpr static double kNumericalEpsilon = 1e-6;
    constexpr static double kMaxConeWidth     = 1e4;   // For SOCP cone constraints
};

}   // namespace backend
}   // namespace planning