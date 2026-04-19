#pragma once
#include "ipopt_solver/ipopt_solver.h"
#include "logger.h"
#include "map.h"
#include "math/polygon2d.h"
#include "params.pb.h"
#include "vehicle_model/kinematic_model.h"
#include <Eigen/Core>
#include <vector>

namespace planning {
namespace backend {

enum VariableIndex
{
    X = 0,
    Y,
    THETA,
    V,
    ACCELERATION,
    STEER_ANGLE,
    MU,       // slack variable for collision avoidance
    LAMBDA,   // slack variable for dynamic feasibility
};

class OBCAFG_eval : public FG_eval {
  public:
    typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> CppMatrixXd;
    typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>              CppVecXd;
    CppAD::AD<double> getCostFunction(const ADvector& x) override;
    ADvector          getConstraints(const ADvector& x) override;
    inline void       setCostWeights(const params::OBCAParams& obca_params) { obca_params_ = obca_params; }
    inline void       setInitParameters(std::shared_ptr<Eigen::VectorXd>& ref_X_ptr, const std::size_t N,
                                        const std::size_t state_num, const std::size_t control_num,
                                        const std::size_t lambda_num, const std::size_t mu_num,
                                        std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
                                        std::shared_ptr<vehicle_model::VehiclePose>&    start_pose,
                                        std::shared_ptr<vehicle_model::VehiclePose>&    goal_pose) {
        ref_X_ptr_         = ref_X_ptr;
        N_                 = N;
        state_num_         = state_num;
        control_num_       = control_num;
        lambda_num_        = lambda_num;
        mu_num_            = mu_num;
        start_pose_ptr_    = start_pose;
        goal_pose_ptr_     = goal_pose;
        dynamic_model_ptr_ = dynamic_model_ptr;
        initializeParameters(dynamic_model_ptr_->GetVehicleParam());
    };

    inline bool initializeParameters(const kinematic_model::VehicleParam& vehicle_param) {
        // Set the vehicle parameters (in body frame with rear axle center at origin)
        // Constraint form: G * c <= g, where c is a point in body frame
        // G matrix defines the 4 half-planes for the vehicle rectangle:
        //   Row 0: [ 1,  0] -> x direction (front constraint)
        //   Row 1: [ 0,  1] -> y direction (left constraint)
        //   Row 2: [-1,  0] -> -x direction (rear constraint)
        //   Row 3: [ 0, -1] -> -y direction (right constraint)
        G_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;

        double half_length = vehicle_param.length() / 2.0;
        double half_width  = vehicle_param.width() / 2.0;

        g_ << half_length, half_width, half_length, half_width;
        return true;
    };   // Initialize the parameters for the OBCA algorithm

  public:
    bool getObstacleBound(const std::shared_ptr<map::Map>& map_ptr);

    bool setConstraintsBound(IpoptSolver::Dvector* xl, IpoptSolver::Dvector* xu, IpoptSolver::Dvector* gl,
                             IpoptSolver::Dvector* gu);

    bool setPoseConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub);
    bool setDynamicConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub);
    bool setControlFeasibleConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub);
    bool setAvoidanceConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub);

    static bool getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
                             Eigen::VectorXd& obstacle_b, std::size_t i);
    static bool getRotationMatrix(const CppAD::AD<double> theta, CppMatrixXd& R);

    static bool getMovementMatrix(const CppAD::AD<double> x, const CppAD::AD<double> y, const CppAD::AD<double> theta,
                                  const double off_set, CppVecXd& t);

  private:
    bool setPoseConstraints(const FG_eval::ADvector& x, ADvector* constraints);
    bool setDynamicConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints);
    bool setControlFeasibleConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints);
    bool setAvoidanceConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints);

    std::shared_ptr<Eigen::VectorXd>               ref_X_ptr_;
    std::size_t                                    N_;             // number of discretization steps
    std::size_t                                    state_num_;     // number of states
    std::size_t                                    control_num_;   // number of controls
    std::size_t                                    lambda_num_           = 0;
    std::size_t                                    mu_num_               = 0;
    std::size_t                                    pose_constraints_num_ = 0;
    std::vector<std::size_t>                       obstable_bound_num_vec_;
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
    std::shared_ptr<vehicle_model::VehiclePose>    start_pose_ptr_;
    std::shared_ptr<vehicle_model::VehiclePose>    goal_pose_ptr_;
    Eigen::MatrixXd                                obstacle_A_;                  // obstacle boundary A
    Eigen::VectorXd                                obstacle_b_;                  // obstacle boundary b
    constexpr static double                        kDt                 = 0.1;    // s
    constexpr static std::size_t                   kVehicleBoundaryNum = 4;      // number of vehicle boundary points
    constexpr static double                        kSafeDist           = 0.5;    // m
    constexpr static double                        kEpsilon            = 1e-4;   // Must match OBCAFG_eval::kEpsilon
    constexpr static double                        kMaxValue           = 1e4;
    params::OBCAParams                             obca_params_;
    Eigen::Matrix<CppAD::AD<double>, kVehicleBoundaryNum, 2> G_;   // ego vehicle matrix: Gx <= g
    Eigen::Matrix<CppAD::AD<double>, kVehicleBoundaryNum, 1> g_;   // Control input matrix
};

class OBCASolver : public IpoptSolver {
    /*
    Solve the optimization problem using the Optimization-Based Collision Avoidance.
    The control u = [a, \sigma] is the control input, where \sigma is the steering angle and a is the acceleration.
    The state x = [x, y, \theta, v] is the state of the vehicle, where x and y are the position coordinates,
    \theta is the heading angle, and v is the velocity.
    */
  public:
    OBCASolver(std::string& options, const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
               const std::shared_ptr<map::Map>& map_ptr, const params::OBCAParams& obca_params, double dt = 0.1)
        : IpoptSolver(options) {
        if (map_ptr == nullptr || dynamic_model_ptr == nullptr) {
            LOG(WARNING) << "The map ptr or dynamic model ptr is null.";
            throw std::runtime_error("The map ptr or dynamic model ptr is null.");
        }
        map_ptr_           = map_ptr;
        dynamic_model_ptr_ = dynamic_model_ptr;
        state_num_         = vehicle_model::KinematicModel::GetStateSize();     // 4: [x, y, theta, v]
        control_num_       = vehicle_model::KinematicModel::GetControlSize();   // 2: [a, sigma]
        fg_eval_           = OBCAFG_eval();
        fg_eval_.setCostWeights(obca_params);
        kDt = dt;
    };
    ~OBCASolver() = default;
    bool Process(const vehicle_model::sdv_path& init_path, std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                 std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);
    inline static vehicle_model::sdv_path Vec3dToSdvPath(const std::vector<Eigen::Vector3d>& path) {
        vehicle_model::sdv_path sdv_path;
        for (const auto& point : path) {
            vehicle_model::VehiclePose pose;
            pose.x     = point.x();
            pose.y     = point.y();
            pose.theta = point.z();
            sdv_path.push_back(pose);
        }
        return sdv_path;
    }
    inline const std::vector<Eigen::Vector4d>& GetStatesResult() const { return states_result_; }
    inline const std::vector<Eigen::Vector2d>& GetControlsResult() const { return controls_result_; }
    inline const std::vector<Eigen::Vector4d>& GetInitStates() const { return initial_states_; }

  private:
    bool setInitVariable(const vehicle_model::sdv_path& init_path, OBCAFG_eval* fg_eval, Dvector* init_variables,
                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    // Helper functions for initializing variables
    bool initializeControlStates(const vehicle_model::sdv_path&               init_path,
                                 std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                 std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    bool initializeDualVariables(OBCAFG_eval* fg_eval, Dvector* init_variables,
                                 std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                 std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    bool setResult(const CppAD::ipopt::solve_result<Dvector>& solution);


    constexpr static double      kEpsilon            = 1e-5;
    double                       kDt                 = 0.1;   // s
    constexpr static std::size_t kVehicleBoundaryNum = 4;
    // initial variables, including: [x_i, y_i, theta_i, v_i, sigma_i, a_i, mu_i, lambda_i], i = 0, ..., N-1
    Eigen::VectorXd x0_;


    double      offset_;        // the distance from the rear axle to the vehicle center
    std::size_t N_;             // Number of discretization steps
    std::size_t state_num_;     // Number of states
    std::size_t control_num_;   // Number of controls

    std::vector<Eigen::Vector4d> states_result_;     // Resulting states after optimization
    std::vector<Eigen::Vector2d> controls_result_;   // Resulting controls after optimization
    std::vector<Eigen::Vector4d> initial_states_;    // Initial states for each time step
    OBCAFG_eval                  fg_eval_;

    std::shared_ptr<map::Map>                      map_ptr_;
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
};
}   // namespace backend
}   // namespace planning
