#include "collision_check/base_check.h"
#include "map/map.h"
#include "params.pb.h"
#include "planner/trajectory_planner.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"
#include <Eigen/Core>
#include <memory>
#include <utility>

namespace solver {

namespace collision_check = planning::collision_check;

struct SolverParams {
    double max_iter;
};

class Solver {
  public:
    Solver();
    ~Solver() = default;

    const problem::PlanRes& Run(const problem::SolverInput& solver_input);

    void                     LoadProblem(const problem::PlanProblem& plan_problem,
                                         const params::SolverParams& solver_params);   // Load problem from protobuf
    const cost_map::CostMap& GetCostMap() const { return map_ptr_->GetCostMap(); };

  private:
    void setMap(std::shared_ptr<map::Map>&& map_ptr) { map_ptr_ = std::move(map_ptr); }
    void setCollisionCheck(std::shared_ptr<collision_check::BaseCheck>&& collision_check_ptr) {
        collision_check_ptr_ = std::move(collision_check_ptr);
    };
    void setPose(const kinematic_model::StateVar& state_var, planning::vehicle_model::VehiclePose& vechile_pose) {
        vechile_pose.x     = state_var.x();
        vechile_pose.y     = state_var.y();
        vechile_pose.theta = state_var.theta();
    };
    void setSolverParams(const params::SolverParams& solver_params) { solver_params_ = solver_params; };
    void setOptTraj(const planning::vehicle_model::opt_states&  opt_states,
                    const planning::vehicle_model::opt_control& opt_controls);
    void setPreOptTraj(const planning::vehicle_model::opt_states&  init_states,
                       const planning::vehicle_model::opt_control& init_controls);
    // define pointer to the submodule
    std::shared_ptr<map::Map>                   map_ptr_;
    std::shared_ptr<collision_check::BaseCheck> collision_check_ptr_;
    std::unique_ptr<planning::TrajPlanner>      traj_planner_ptr_;

    planning::vehicle_model::VehiclePose start_vec_;   // [x, y, theta]
    planning::vehicle_model::VehiclePose goal_vec_;
    problem::PlanRes                     plan_res_;
    params::SolverParams                 solver_params_;
};

}   // namespace solver