/*
 * @Author: wenqing-2021 yuansj@hnu.edu.cn
 * @Date: 2026-01-31 12:14:24
 * @LastEditors: wenqing-2021 yuansj@hnu.edu.cn
 * @LastEditTime: 2026-03-22 04:02:15
 * @FilePath: /AutomatedPark/src/planner/trajectory_planner.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "admm/admm_planner.h"
#include "base_check.h"
#include "hybrid_a_star/hybrid_a_star.h"
#include "obca/obca_planner.h"
#include "speed_planner/piece_wise_jerk.h"

#include <string>

namespace planning {
using pwjspeed = backend::PiecewiseJerkSpeedOptimizer;
using admmopt  = backend::ADMMSolver;
using obcaopt  = backend::OBCASolver;
class TrajPlanner {
  public:
    TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                const kinematic_model::VehicleParam&               vehicle_param);
    ~TrajPlanner() = default;

    bool                Process(const vehicle_model::VehiclePose& start_vec,
                                const vehicle_model::VehiclePose& goal_vec);   // main function to generate the trajectory
    inline const double GetInitPathLength() { return frontend_path_length_; };               // get the init path length
    inline std::vector<Eigen::Vector3d>& GetDebugNodeList() { return debug_node_list_; };    // get the debug node list
    const vehicle_model::opt_status&     GetOptStates() const { return opt_status_; };       // get the opt states
    const vehicle_model::opt_control&    GetOptControls() const { return opt_controls_; };   // get the opt controls

  private:
    void initObcaSolver(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr);
    bool runBackendOpt(const std::vector<Eigen::Vector3d>* const frontend_path,
                       const vehicle_model::VehiclePose& start_vec, const vehicle_model::VehiclePose& goal_vec);
    bool setStatusControls(const std::vector<Eigen::Vector4d>& opt_traj);

    std::unique_ptr<frontend::HybridAstar>         hybrid_astar_ptr_;   // pointer to the hybrid A* planner
    std::unique_ptr<pwjspeed>                      pwj_speed_ptr_;
    std::unique_ptr<admmopt>                       admm_solver_ptr_;   // pointer to the ADMM solver
    std::unique_ptr<obcaopt>                       obca_solver_ptr_;   // pointer to the OBCA solver
    std::shared_ptr<vehicle_model::KinematicModel> vehicle_model_ptr_;
    std::vector<Eigen::Vector3d>                   debug_node_list_;
    std::string                                    backend_solver_;
    double                                         frontend_path_length_{0.0};
    vehicle_model::opt_status                      opt_status_;
    vehicle_model::opt_control                     opt_controls_;
    constexpr static double                        kEpsilon = 1e-5;

};   // class BasePlanner

}   // namespace planning