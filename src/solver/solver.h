#include "collision_check/base_check.h"
#include "map/map.h"
#include "params.pb.h"
#include "planner/trajectory_planner.h"
#include "problem.pb.h"
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
    const cost_map::CostMap& getCostMap() const { return map_ptr_->GetCostMap(); };

  private:
    void setMap(std::shared_ptr<map::Map>&& map_ptr) { map_ptr_ = std::move(map_ptr); }
    void setCollisionCheck(std::shared_ptr<collision_check::BaseCheck>&& collision_check_ptr) {
        collision_check_ptr_ = std::move(collision_check_ptr);
    };
    void setStart(const Eigen::Vector3d& start_vec) { start_vec_ = start_vec; };
    void setGoal(const Eigen::Vector3d& goal_vec) { goal_vec_ = goal_vec; };
    void setSolverParams(const params::SolverParams& solver_params) { solver_params_ = solver_params; };

    // define pointer to the submodule
    std::shared_ptr<map::Map>                   map_ptr_;
    std::shared_ptr<collision_check::BaseCheck> collision_check_ptr_;
    std::unique_ptr<planning::TrajPlanner>      traj_planner_ptr_;

    Eigen::Vector3d      start_vec_;   // [x, y, theta]
    Eigen::Vector3d      goal_vec_;
    problem::PlanRes     plan_res_;
    params::SolverParams solver_params_;
};

}   // namespace solver