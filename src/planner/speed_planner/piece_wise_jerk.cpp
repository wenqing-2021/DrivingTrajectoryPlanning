#include "piece_wise_jerk.h"
#include "common/math/math_utils.h"
#include "logger/logger.h"
#include "planner/vehicle_model/kinematic_model.h"
namespace planning {
namespace backend {

PiecewiseJerkSpeedOptimizer::PiecewiseJerkSpeedOptimizer(const params::PiecewiseJerkParams& pwj_params)
    : pwj_params_(pwj_params){
          // 1. set the piecewise jerk speed optimizer config
          // setParams(config);
          // 2. set the piecewise jerk speed optimizer solver
          // setSolver();
      };

void PiecewiseJerkSpeedOptimizer::Optimize(const std::vector<Eigen::Vector3d>& path,
                                           const Eigen::Vector3d&              vehicle_pose) {
    result_path_ = path;
    std::vector<std::vector<Eigen::Vector3d>> shift_seg_paths;
    // 1. get the shift segment paths
    getShiftSegPaths(path, &shift_seg_paths);
    LOG(INFO) << "The shift segment paths size is: " << shift_seg_paths.size();
    // 2. optimize the speed for each segment and fill the result path
    std::size_t start_index = 0;
    double      init_path_head =
        common::math::NormalizeAngle(std::atan2(path[0].y() - vehicle_pose.y(), path[0].x() - vehicle_pose.x()));
    double speed_direct = getSpeedDirect(init_path_head, vehicle_pose.z());
    for (std::size_t i = 0; i < shift_seg_paths.size(); ++i) {
        // 2.1 optimize the speed
        if (!optimizeSpeed(shift_seg_paths[i])) {
            // 2.2 if optimize failed, return
            LOG(INFO) << "Failed to optimize the speed for segment " << i;
            return;
        } else {
            // 2.3 if optimize success, fill the result path
            for (std::size_t j = 0; j < shift_seg_paths[i].size(); ++j) {
                shift_seg_paths[i][j].z()     = speed_direct * shift_seg_paths[i][j].z();
                result_path_[start_index + j] = shift_seg_paths[i][j];
            }
            speed_direct = -speed_direct;
            start_index += shift_seg_paths[i].size();
        }
    }
};

void PiecewiseJerkSpeedOptimizer::getShiftSegPaths(const std::vector<Eigen::Vector3d>&        path,
                                                   std::vector<std::vector<Eigen::Vector3d>>* shift_seg_paths) {
    // 1. convert the vector to the eigen matrix with path vector
    Eigen::MatrixXd path_vector(path.size() - 1, 2);
    for (std::size_t i = 1; i < path.size(); ++i) {
        path_vector(i, 0) = path[i].x() - path[i - 1].x();
        path_vector(i, 1) = path[i].y() - path[i - 1].y();
    }
    // 2. Compute the dot product of consecutive vectors
    Eigen::VectorXd dot_products =
        (path_vector.topRows(path_vector.rows() - 1).array() * path_vector.bottomRows(path_vector.rows() - 1).array())
            .rowwise()
            .sum();
    // 3. Find each segment paths
    shift_seg_paths->clear();
    int start_index = 0;
    for (int i = 0; i < dot_products.size(); ++i) {
        if (dot_products(i) < 0) {
            std::vector<Eigen::Vector3d> seg_path;
            for (int j = start_index; j <= i + 1; ++j) { seg_path.push_back(path[j]); }
            shift_seg_paths->push_back(seg_path);
            start_index = i + 1;
        }
    };
    // 4. Add the last segment path
    std::vector<Eigen::Vector3d> Last_seg_path;
    for (int j = start_index; j < path.size(); ++j) { Last_seg_path.push_back(path[j]); }
    shift_seg_paths->push_back(Last_seg_path);
}

double PiecewiseJerkSpeedOptimizer::getSpeedDirect(const double init_path_head, const double vehicle_head) {
    if (common::math::AngleDiff(vehicle_head, init_path_head) > M_PI_2) {
        return -1.0;
    } else {
        return 1.0;
    }
}

bool PiecewiseJerkSpeedOptimizer::optimizeSpeed(const std::vector<Eigen::Vector3d>& path) {
    return true;
}
}   // namespace backend
}   // namespace planning
