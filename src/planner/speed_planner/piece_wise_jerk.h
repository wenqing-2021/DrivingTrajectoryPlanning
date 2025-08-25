#include "Eigen/Core"
#include "params.pb.h"
#include "qp_solver/qp_solver.h"
#include "vehicle_model/kinematic_model.h"
namespace planning {
namespace backend {

class PiecewiseJerkSpeedOptimizer {
  public:
    PiecewiseJerkSpeedOptimizer(const params::PiecewiseJerkParams& pwj_params);
    ~PiecewiseJerkSpeedOptimizer() = default;

    bool Optimize(const std::vector<Eigen::Vector3d>& path, const vehicle_model::VehiclePose& vehicle_pose);
    inline const std::vector<Eigen::Vector3d>* const GetResult() const { return &result_traj_; };

    const double      GetDt() const { return kDt; };
    const std::size_t GetSegmentNum() const { return segment_num_; };

  private:
    bool   optimizeSpeed(const std::vector<Eigen::Vector3d>& path);
    bool   getShiftSegPaths(const std::vector<Eigen::Vector3d>&        path,
                            std::vector<std::vector<Eigen::Vector3d>>* shift_seg_paths);
    bool   initVariables(const std::vector<Eigen::Vector3d>& path);
    bool   initHessianGradient(const std::vector<Eigen::Vector3d>& path);
    bool   initBounds(const std::vector<Eigen::Vector3d>& path);
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
    std::size_t                  segment_num_ = 0;   // the number of segments
    constexpr static double      kDt          = 0.1;
    constexpr static double      kDt2         = kDt * kDt;
    constexpr static double      kEpsilon     = 1e-3;
};

}   // namespace backend
}   // namespace planning