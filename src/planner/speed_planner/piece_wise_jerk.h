#include "Eigen/Core"
#include "params.pb.h"
namespace planning {
namespace backend {

class PiecewiseJerkSpeedOptimizer {
  public:
    PiecewiseJerkSpeedOptimizer(const params::PiecewiseJerkParams& pwj_params);
    ~PiecewiseJerkSpeedOptimizer() = default;

    void Optimize(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& vehicle_pose);

  private:
    bool   optimizeSpeed(const std::vector<Eigen::Vector3d>& path);
    void   getShiftSegPaths(const std::vector<Eigen::Vector3d>&        path,
                            std::vector<std::vector<Eigen::Vector3d>>* shift_seg_paths);
    void   initVariables(std::size_t N);
    void   buildObjective();
    void   buildConstraint();
    double getSpeedDirect(const double init_path_head, const double vehicle_head);

    std::vector<Eigen::Vector3d> result_path_;
    params::PiecewiseJerkParams  pwj_params_;
};

}   // namespace backend
}   // namespace planning