#pragma once

#include "eicos.hpp"
#include "logger.h"
#include "map.h"
#include "params.pb.h"
#include "vehicle_model/kinematic_model.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

namespace planning {
namespace backend {

enum class VariableIdx : std::size_t
{
    X = 0,
    Y,
    THETA,
    V,
    ACCELERATION,
    STEER_ANGLE,
};

enum class DualVariableIdx : std::size_t
{
    LAMBDA = 0,   // for dynamic constraints
    MU,           // for collision avoidance constraints
    Z,
};

struct RDAOptimizationResult {
    std::vector<Eigen::Vector4d> states;     // [x, y, theta, v]
    std::vector<Eigen::Vector2d> controls;   // [acceleration, steering_angle]
    double                       cost;
    int                          iterations;
    bool                         is_feasible;
};

struct RDAObstacleWorkspace {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::MatrixXd lambda;
    Eigen::MatrixXd mu;
    Eigen::VectorXd z;
    Eigen::MatrixXd xi;
    Eigen::VectorXd zeta;
    Eigen::MatrixXd obsA_lam;
    Eigen::VectorXd obsb_lam;
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
    // Iterative ADMM-based solving
    bool iterativeSolve(const vehicle_model::sdv_path&               init_path,
                        std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                        std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    // Initialize optimization variables
    bool setInitVariable(const vehicle_model::sdv_path&               init_path,
                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    // Helper: Initialize state and control variables
    bool initializeStateControl(const vehicle_model::sdv_path& init_path);

    bool initializeDualVariables();

    // Set constraint bounds
    bool setVariableBounds(Eigen::VectorXd& x_lower, Eigen::VectorXd& x_upper);

    bool setDynamicConstraints(Eigen::MatrixXd& A_constraint, Eigen::VectorXd& b_constraint);

    bool setCollisionAvoidanceConstraints(Eigen::MatrixXd& A_collision, std::vector<Eigen::VectorXd>& b_collision_vec);

    bool constructCostAndConstraints(Eigen::MatrixXd& P, Eigen::VectorXd& q, Eigen::MatrixXd& Aeq, Eigen::VectorXd& beq,
                                     Eigen::MatrixXd& G, Eigen::VectorXd& h);
    bool initializeAdmmWorkspace(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& Aeq,
                                 const Eigen::VectorXd& beq, const Eigen::MatrixXd& G, const Eigen::VectorXd& h);
    bool runAdmmIterations();
    bool solveStateControlStepWithEicos();
    bool solveLambdaMuZStepWithEicos(double& dual_residual);
    bool solveObstacleLambdaMuZWithEicos(std::size_t obs_index, double& residual);
    double updateXiZeta();
    void   setOptimalResultFromAdmm();
    bool   initializeRDAWorkspace();
    void   updateObstacleDualProducts();
    double computeIm(const std::size_t obs_index, const std::size_t time_index, const Eigen::VectorXd& variables) const;
    Eigen::RowVector2d computeHm(const std::size_t obs_index, const std::size_t time_index,
                                 const Eigen::VectorXd& variables) const;
    double             getDistanceVariable(const Eigen::VectorXd& variables, const std::size_t time_index) const;
    std::size_t        getDistanceIndex(const std::size_t time_index) const;

    static std::size_t Idx(VariableIdx idx);
    static double      SoftThreshold(double value, double threshold);
    static bool        ComputeObstacleHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
                                                Eigen::VectorXd& obstacle_b);

    // Cost function setup
    bool setCostFunction(Eigen::MatrixXd& P, Eigen::VectorXd& q);
    bool setReferenceCost(Eigen::MatrixXd& P, Eigen::VectorXd& q);

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
    Eigen::VectorXd       dual_variables_;
    Eigen::VectorXd       xi_;   // for penalty coefficients
    Eigen::VectorXd       zeta_;
    Eigen::VectorXd       optimal_solution_;
    RDAOptimizationResult optimization_result_;

    std::vector<RDAObstacleWorkspace> rda_obstacles_;
    Eigen::Matrix<double, 4, 2>       vehicle_G_;
    Eigen::Matrix<double, 4, 1>       vehicle_h_;

    Eigen::MatrixXd admm_P_;
    Eigen::MatrixXd admm_Aeq_;
    Eigen::MatrixXd admm_G_;
    Eigen::VectorXd admm_q_;
    Eigen::VectorXd admm_beq_;
    Eigen::VectorXd admm_h_;
    Eigen::VectorXd admm_x_;
    Eigen::VectorXd admm_z_;
    Eigen::VectorXd admm_lambda_;
    Eigen::VectorXd admm_mu_;
    Eigen::VectorXd admm_u_;
    double          admm_rho_{1.0};
    double          admm_l1_weight_{1e-2};
    std::size_t     admm_var_num_{0};
    std::size_t     admm_constraint_num_{0};
    std::size_t     distance_dim_{0};
    std::size_t     distance_offset_{0};

    // Results storage
    std::vector<Eigen::Vector4d> states_result_;
    std::vector<Eigen::Vector2d> controls_result_;
    std::vector<Eigen::Vector4d> initial_states_;
    std::vector<Eigen::Vector2d> initial_controls_;

    // Algorithm parameters
    double              convergence_tolerance_;
    int                 max_rda_iterations_;
    double              penalty_weight_;
    double              l1_weight_;
    bool                use_warm_start_;
    std::vector<double> cost_history_;   // For convergence monitoring
    double              min_safety_distance_{0.1};
    double              max_safety_distance_{1.0};
    double              slack_gain_{8.0};
    double              hm_penalty_weight_{1.0};
    double              im_penalty_weight_{1.0};

    // Vehicle constraints
    double max_velocity_;
    double max_acceleration_;
    double max_steering_angle_;
    double wheel_base_;
    double vehicle_length_;
    double vehicle_width_;

    // Safety and numerical parameters
    constexpr static double      kSafetyMargin       = 0.3;   // m
    constexpr static std::size_t kVehicleBoundaryNum = 4;
    constexpr static double      kNumericalEpsilon   = 1e-6;
    constexpr static double      kMaxConeWidth       = 1e4;   // For SOCP cone constraints
};

}   // namespace backend
}   // namespace planning