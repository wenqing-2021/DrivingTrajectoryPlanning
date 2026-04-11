/*
 * @Author: wenqing-2021 yuansj@hnu.edu.cn
 * @Date: 2026-01-31 12:14:24
 * @LastEditors: wenqing-2021 yuansj@hnu.edu.cn
 * @LastEditTime: 2026-04-11 08:46:30
 * @Description: trajectory planner class definition
 */
#pragma once
#include "base_check.h"
#include "hybrid_a_star/hybrid_a_star.h"
#include "obca/obca_planner.h"
#include "rda/rda_planner.h"
#include "speed_planner/piece_wise_jerk.h"

#include <string>

namespace planning {
using pwjspeed = backend::PiecewiseJerkSpeedOptimizer;
using rdaopt   = backend::RDASolver;
using obcaopt  = backend::OBCASolver;
class TrajPlanner {
  public:
    TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                const kinematic_model::VehicleParam&               vehicle_param);
    ~TrajPlanner() = default;

    bool          Process(const vehicle_model::VehiclePose& start_vec,
                          const vehicle_model::VehiclePose& goal_vec);     // main function to generate the trajectory
    inline double GetInitPathLength() { return frontend_path_length_; };   // get the init path length
    inline std::vector<Eigen::Vector3d>& GetDebugNodeList() { return debug_node_list_; };    // get the debug node list
    const vehicle_model::opt_states&     GetOptStates() const { return opt_states_; };       // get the opt states
    const vehicle_model::opt_control&    GetOptControls() const { return opt_controls_; };   // get the opt controls
    const vehicle_model::opt_states&     GetInitStates() const { return init_states_; };     // get the init states
    const vehicle_model::opt_control&    GetInitControls() const { return init_controls_; };   // get the init controls

  private:
    void initObcaSolver(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr);
    bool runBackendOpt(const std::vector<Eigen::Vector3d>* const frontend_path,
                       const vehicle_model::VehiclePose& start_vec, const vehicle_model::VehiclePose& goal_vec);
    bool setstatesControls(vehicle_model::opt_states& opt_states, vehicle_model::opt_control& opt_controls,
                           const std::vector<Eigen::Vector4d>& opt_traj);

    std::unique_ptr<frontend::HybridAstar>         hybrid_astar_ptr_;   // pointer to the hybrid A* planner
    std::unique_ptr<pwjspeed>                      pwj_speed_ptr_;
    std::unique_ptr<rdaopt>                        rda_solver_ptr_;    // pointer to the RDA solver
    std::unique_ptr<obcaopt>                       obca_solver_ptr_;   // pointer to the OBCA solver
    std::shared_ptr<vehicle_model::KinematicModel> vehicle_model_ptr_;
    std::vector<Eigen::Vector3d>                   debug_node_list_;
    std::string                                    backend_solver_;
    std::vector<Eigen::Vector4d>                   init_traj_;   // initial trajectory for optimization
    double                                         frontend_path_length_{0.0};
    vehicle_model::opt_states                      opt_states_;
    vehicle_model::opt_control                     opt_controls_;
    vehicle_model::opt_states                      init_states_;
    vehicle_model::opt_control                     init_controls_;
    constexpr static double                        kEpsilon = 1e-5;

};   // class BasePlanner

}   // namespace planning