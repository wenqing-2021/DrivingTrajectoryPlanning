#pragma once
#include "ipopt_solver/ipopt_solver.h"
#include "logger.h"
#include "map.h"
#include "math/polygon2d.h"
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
    STEER_ANGLE,
    ACCELERATION,
    MU,       // slack variable for collision avoidance
    LAMBDA,   // slack variable for dynamic feasibility
};

class OBCAFG_eval : public FG_eval {
  public:
    typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> CppMatrixXd;
    typedef Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>              CppVecXd;
    CppAD::AD<double> getCostFunction(const ADvector& x) override;
    ADvector          getConstraints(const ADvector& x) override;
    inline const void setInitParameters(std::shared_ptr<Eigen::VectorXd>& ref_X_ptr, const std::size_t N,
                                        const std::size_t state_num, const std::size_t control_num,
                                        std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
                                        std::shared_ptr<vehicle_model::VehiclePose>&    start_pose,
                                        std::shared_ptr<vehicle_model::VehiclePose>&    goal_pose) {
        ref_X_ptr_         = ref_X_ptr;
        N_                 = N;
        state_num_         = state_num;
        control_num_       = control_num;
        start_pose_ptr_    = start_pose;
        goal_pose_ptr_     = goal_pose;
        dynamic_model_ptr_ = dynamic_model_ptr;
        initializeParameters(dynamic_model_ptr_->GetVehicleParam());
    };

    inline bool initializeParameters(const kinematic_model::VehicleParam& vehicle_param) {
        // Set the vehicle parameters
        G_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
        g_ << vehicle_param.length() / 2, vehicle_param.width() / 2, vehicle_param.length() / 2,
            vehicle_param.width() / 2;
        return true;
    };   // Initialize the parameters for the OBCA algorithm

  public:
    bool getObstacleBound(const std::shared_ptr<map::Map>& map_ptr);

    static bool getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
                             Eigen::VectorXd& obstacle_b, std::size_t i);
    static bool getRotationMatrix(const CppAD::AD<double> theta, CppMatrixXd& R);

    static bool getMovementMatrix(const CppAD::AD<double> x, const CppAD::AD<double> y, const CppAD::AD<double> theta,
                                  const double off_set, CppVecXd& t);

  private:
    bool getPoseConstraints(const FG_eval::ADvector& x, ADvector* constraints, IpoptSolver::Dvector* lb,
                            IpoptSolver::Dvector* ub);
    bool getDynamicConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints, IpoptSolver::Dvector* lb,
                               IpoptSolver::Dvector* ub);
    bool getControlFeasibleConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints,
                                       IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub);
    bool getAvoidanceConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints, IpoptSolver::Dvector* lb,
                                 IpoptSolver::Dvector* ub) const;

    std::shared_ptr<Eigen::VectorXd>               ref_X_ptr_;
    std::size_t                                    N_;             // number of discretization steps
    std::size_t                                    state_num_;     // number of states
    std::size_t                                    control_num_;   // number of controls
    std::vector<std::size_t>                       obstable_bound_num_vec_;
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
    std::shared_ptr<vehicle_model::VehiclePose>    start_pose_ptr_;
    std::shared_ptr<vehicle_model::VehiclePose>    goal_pose_ptr_;
    Eigen::MatrixXd                                obstacle_A_;                  // obstacle boundary A
    Eigen::VectorXd                                obstacle_b_;                  // obstacle boundary b
    constexpr static double                        kDt                 = 0.1;    // s
    constexpr static std::size_t                   kVehicleBoundaryNum = 4;      // number of vehicle boundary points
    constexpr static double                        kSafeDist           = 0.05;   // m
    constexpr static double                        kEpsilon            = 1e-4;
    constexpr static double                        kMaxValue           = 1e4;
    Eigen::Matrix<CppAD::AD<double>, kVehicleBoundaryNum, 2> G_;   // ego vehicle matrix: Gx <= g
    Eigen::Matrix<CppAD::AD<double>, kVehicleBoundaryNum, 1> g_;   // Control input matrix
};

class OBCASolver : public IpoptSolver {
    /*
    Solve the optimization problem using the Optimization-Based Collision Avoidance.
    The control u = [\sigma, a] is the control input, where \sigma is the steering angle and a is the acceleration.
    The state x = [x, y, \theta, v] is the state of the vehicle, where x and y are the position coordinates,
    \theta is the heading angle, and v is the velocity.
    */
  public:
    OBCASolver(std::string& options, const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
               const std::shared_ptr<map::Map>& map_ptr)
        : IpoptSolver(options) {
        if (map_ptr == nullptr || dynamic_model_ptr == nullptr) {
            LOG(WARNING) << "The map ptr or dynamic model ptr is null.";
            throw std::runtime_error("The map ptr or dynamic model ptr is null.");
        }
        map_ptr_           = map_ptr;
        dynamic_model_ptr_ = dynamic_model_ptr;
        state_num_         = vehicle_model::KinematicModel::GetStateSize();     // 4: [x, y, theta, v]
        control_num_       = vehicle_model::KinematicModel::GetControlSize();   // 2: [sigma, a]
        fg_eval_           = OBCAFG_eval();
    };
    ~OBCASolver() = default;
    bool Process(const vehicle_model::sdv_path& init_path, std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                 std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);

    inline const std::vector<Eigen::Vector4d>& GetStatesResult() const { return states_result_; }
    inline const std::vector<Eigen::Vector2d>& GetControlsResult() const { return controls_result_; }

  private:
    bool setInitVariable(const vehicle_model::sdv_path& init_path, OBCAFG_eval* fg_eval, Dvector* init_variables,
                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr);


    constexpr static double      kEpsilon            = 1e-5;
    constexpr static double      kDt                 = 0.1;   // s
    constexpr static std::size_t kVehicleBoundaryNum = 4;
    // initial variables, including: [x_i, y_i, theta_i, v_i, sigma_i, a_i, mu_i, lambda_i], i = 0, ..., N-1
    Eigen::VectorXd x0_;


    double      offset_;        // the distance from the rear axle to the vehicle center
    std::size_t N_;             // Number of discretization steps
    std::size_t state_num_;     // Number of states
    std::size_t control_num_;   // Number of controls

    std::vector<Eigen::Vector4d> states_result_;     // Resulting states after optimization
    std::vector<Eigen::Vector2d> controls_result_;   // Resulting controls after optimization
    OBCAFG_eval                  fg_eval_;

    std::shared_ptr<map::Map>                      map_ptr_;
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
};
}   // namespace backend
}   // namespace planning
