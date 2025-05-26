#pragma once
#include "collision_check/base_check.h"
#include "planner/hybrid_a_star/hybrid_a_star.h"
#include "planner/speed_planner/piece_wise_jerk.h"
#include "vehicle_model/kinematic_model.h"
#include <string>

namespace planning {
using pwjspeed = backend::PiecewiseJerkSpeedOptimizer;
class TrajPlanner {
  public:
    TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                const kinematic_model::VehicleParam&               vehicle_param);
    ~TrajPlanner() = default;

    bool                Process(const Eigen::Vector3d& start_vec,
                                const Eigen::Vector3d& goal_vec);   // main function to generate the trajectory
    inline const double GetInitPathLength() { return frontend_path_length_; };              // get the init path length
    inline std::vector<Eigen::Vector3d>& GetDebugNodeList() { return debug_node_list_; };   // get the debug node list
    const vehicle_model::opt_status&     GetInitStates() const { return init_states_; };    // get the init states
    const vehicle_model::opt_control&    GetInitControls() const { return init_controls_; };   // get the init controls

  private:
    bool setStatusControls(const std::vector<Eigen::Vector3d>* const init_path_ptr,
                           const std::vector<Eigen::Vector3d>* const init_traj_ptr);

    std::unique_ptr<frontend::HybridAstar>         hybrid_astar_ptr_;   // pointer to the hybrid A* planner
    std::unique_ptr<pwjspeed>                      pwj_speed_ptr_;
    std::shared_ptr<kinematic_model::VehicleParam> vehicle_param_ptr_;
    std::vector<Eigen::Vector3d>                   debug_node_list_;
    double                                         frontend_path_length_{0.0};
    vehicle_model::opt_status                      init_states_;
    vehicle_model::opt_control                     init_controls_;
    constexpr static double                        kEpsilon = 1e-5;

};   // class BasePlanner

}   // namespace planning