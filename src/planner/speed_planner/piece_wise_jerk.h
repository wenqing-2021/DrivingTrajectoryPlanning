#include "Eigen/Core"
#include "params.pb.h"
#include "planner/qp_solver/qp_solver.h"
namespace planning {
namespace backend {

class PiecewiseJerkSpeedOptimizer {
  public:
    PiecewiseJerkSpeedOptimizer(const params::PiecewiseJerkParams& pwj_params);
    ~PiecewiseJerkSpeedOptimizer() = default;

    bool Optimize(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& vehicle_pose);
    inline const std::vector<Eigen::Vector3d>* const GetResult() const { return &result_traj_; };

  private:
    bool   optimizeSpeed(const std::vector<Eigen::Vector3d>& path);
    bool   getShiftSegPaths(const std::vector<Eigen::Vector3d>&        path,
                            std::vector<std::vector<Eigen::Vector3d>>* shift_seg_paths);
    bool   initVariables(const std::vector<Eigen::Vector3d>& path);
    bool   initHessianGradient(const std::vector<Eigen::Vector3d>& path);
    bool   initBounds(const std::vector<Eigen::Vector3d>& path);
    void   buildObjective();
    void   buildConstraint();
    double getSpeedDirect(const double init_path_head, const double vehicle_head);

    std::unique_ptr<QPSolver> qp_solver_ptr_ = nullptr;

    Eigen::VectorXd init_variables_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;
    Eigen::MatrixXd hessian_matrix_;
    Eigen::MatrixXd linear_matrix_;

    std::vector<Eigen::Vector3d> result_traj_;
    params::PiecewiseJerkParams  pwj_params_;
    constexpr static double      kDt      = 0.1;
    constexpr static double      kDt2     = kDt * kDt;
    constexpr static double      kEpsilon = 1e-3;
};

}   // namespace backend
}   // namespace planning