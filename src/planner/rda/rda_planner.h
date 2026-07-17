#pragma once

#include "eicos.hpp"
#include "map/map.h"
#include "math/polygon2d.h"
#include "params.pb.h"
#include "qp_solver/qp_solver.h"
#include "vehicle_model/kinematic_model.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <future>
#include <memory>
#include <vector>

namespace planning {
namespace backend {

struct RDAObstacle {
    std::vector<Eigen::MatrixXd> A_list;   // T+1 size, each max_edge_num x 2
    std::vector<Eigen::VectorXd> b_list;   // T+1 size, each max_edge_num x 1
};

struct RDADualVariables {
    Eigen::MatrixXd lam;    // max_edge_num x (T+1)
    Eigen::MatrixXd mu;     // 4 x (T+1) (assuming vehicle is modeled as 4 lines)
    Eigen::VectorXd z;      // 1 x T
    Eigen::MatrixXd xi;     // (T+1) x 2
    Eigen::VectorXd zeta;   // 1 x T
};

class RDASolver {
  public:
    RDASolver(const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
              const std::shared_ptr<map::Map>& map_ptr, const params::RDAParams& rda_params, double dt = 0.1);
    ~RDASolver() = default;

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

    const std::vector<Eigen::Vector4d>& GetStatesResult() const { return opt_traj_; }
    const std::vector<Eigen::Vector4d>& GetInitialStates() const { return init_traj_; }

  private:
    void getObstaclesFromMap();
    bool solveSU(const vehicle_model::VehiclePose& start_pose, const vehicle_model::VehiclePose& goal_pose);
    bool solveLamMuZ();
    void updateMultipliers();
    void computeLinearizedDynamics(double v, double delta, double theta, Eigen::MatrixXd& A, Eigen::MatrixXd& B,
                                   Eigen::VectorXd& C);

    // Member variables
    std::shared_ptr<vehicle_model::KinematicModel> dynamic_model_ptr_;
    std::shared_ptr<map::Map>                      map_ptr_;
    params::RDAParams                              rda_params_;
    double                                         dt_;
    std::size_t                                    T_;
    std::size_t                                    max_edge_num_ = 5;

    std::vector<RDAObstacle>      obstacles_;
    std::vector<RDADualVariables> duals_;

    Eigen::MatrixXd state_traj_;     // 4 x (T+1)
    Eigen::MatrixXd control_traj_;   // 2 x T
    Eigen::VectorXd slack_d_;        // 1 x T

    std::vector<Eigen::Vector4d> opt_traj_;
    std::vector<Eigen::Vector4d> init_traj_;

    // Vehicle collision constraints
    Eigen::MatrixXd G_vehicle_;   // 4 x 2
    Eigen::VectorXd h_vehicle_;   // 4 x 1
    double          L_;           // wheelbase
};

}   // namespace backend
}   // namespace planning