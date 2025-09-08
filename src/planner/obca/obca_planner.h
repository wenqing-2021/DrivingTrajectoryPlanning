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
    CppAD::AD<double> getCostFunction(const ADvector& x) override;
    ADvector          getConstraints(const ADvector& x) override;
    inline const void setInitParameters(const Eigen::VectorXd& ref_X, const std::size_t N, const std::size_t state_num,
                                        const std::size_t control_num) {
        ref_X_       = ref_X;
        N_           = N;
        state_num_   = state_num;
        control_num_ = control_num;
    };

  private:
    bool getPoseConstraints(const ADvector& start_pose, const ADvector& goal_pose, const ADvector& x,
                            FG_eval::ADvector* constraints, FG_eval::ADvector* lb, FG_eval::ADvector* ub);
    bool getDynamicConstraints(const ADvector& x, FG_eval::ADvector* constraints, FG_eval::ADvector* lb,
                               FG_eval::ADvector*                                    ub,
                               const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr);
    bool getControlFeasibleConstraints(const ADvector& x, FG_eval::ADvector* constraints, FG_eval::ADvector* lb,
                                       FG_eval::ADvector*                                    ub,
                                       const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr);
    bool getAvoidanceConstraints(const ADvector& x, FG_eval::ADvector* constraints, FG_eval::ADvector* lb,
                                 FG_eval::ADvector* ub);

    Eigen::VectorXd         ref_X_;
    std::size_t             N_;             // number of discretization steps
    std::size_t             state_num_;     // number of states
    std::size_t             control_num_;   // number of controls
    constexpr static double kDt = 0.1;      // s
};

class OBCASolver : public IpoptSolver {
    /*
    Solve the optimization problem using the Optimization-Based Collision Avoidance.
    The control u = [\sigma, a] is the control input, where \sigma is the steering angle and a is the acceleration.
    The state x = [x, y, \theta, v] is the state of the vehicle, where x and y are the position coordinates,
    \theta is the heading angle, and v is the velocity.
    */
  public:
    OBCASolver(std::string& options, const std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr,
               const std::shared_ptr<map::Map> map_ptr)
        : IpoptSolver(options) {
        if (map_ptr == nullptr || dynamic_model_ptr == nullptr) {
            LOG(WARNING) << "The map ptr or dynamic model ptr is null.";
        }
        map_ptr_           = map_ptr;
        dynamic_model_ptr_ = dynamic_model_ptr;
        initializeParameters(dynamic_model_ptr_->GetVehicleParam());
    };
    ~OBCASolver() = default;
    bool Process(const vehicle_model::sdv_path& init_path, const vehicle_model::VehiclePose& start_pose,
                 const vehicle_model::VehiclePose& goal_pose);

    inline const std::vector<Eigen::Vector4d>& GetStatesResult() const { return states_result_; }
    inline const std::vector<Eigen::Vector2d>& GetControlsResult() const { return controls_result_; }

    static bool getObstacleBound(const std::shared_ptr<map::Map> map_ptr, Eigen::MatrixXd& obstacle_A,
                                 Eigen::VectorXd& obstacle_b);

    static bool getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
                             Eigen::VectorXd& obstacle_b, std::size_t i);
    bool        getRotationMatrix(const double theta, Eigen::Matrix2d& R);

    bool getMovementMatrix(const double x, const double y, const double theta, Eigen::Matrix<double, 2, 1>& t);

  private:
    bool setInitVariable(const vehicle_model::sdv_path& init_path, OBCAFG_eval* fg_eval);
    bool initializeParameters(
        const kinematic_model::VehicleParam& vehicle_param);   // Initialize the parameters for the OBCA algorithm

    constexpr static double      kEpsilon            = 1e-5;
    constexpr static double      kDt                 = 0.1;   // s
    constexpr static std::size_t kVehicleBoundaryNum = 4;
    // initial variables, including: [x_i, y_i, theta_i, v_i, sigma_i, a_i, mu_i, lambda_i], i = 0, ..., N-1
    Eigen::VectorXd                               x0_;
    Eigen::Matrix<double, kVehicleBoundaryNum, 2> G_;   // ego vehicle matrix: Gx <= g
    Eigen::Matrix<double, kVehicleBoundaryNum, 1> g_;   // Control input matrix


    double      offset_;        // the distance from the rear axle to the vehicle center
    std::size_t N_;             // Number of discretization steps
    std::size_t state_num_;     // Number of states
    std::size_t control_num_;   // Number of controls

    std::vector<Eigen::Vector4d> states_result_;     // Resulting states after optimization
    std::vector<Eigen::Vector2d> controls_result_;   // Resulting controls after optimization

    std::shared_ptr<map::Map>                      map_ptr_;
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
};
}   // namespace backend
}   // namespace planning
