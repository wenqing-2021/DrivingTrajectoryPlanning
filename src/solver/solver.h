#include "logger.h"
#include "map.h"
#include "problem.pb.h"
#include <Eigen/Core>
#include <memory>

namespace solver {

struct SolverParams {
    double max_iter;
};

class Solver {
  public:
    Solver();
    ~Solver() = default;

    const problem::PlanRes& run(const problem::SolverInput& solver_input);

    void load_problem(const problem::PlanProblem& plan_problem);   // Load problem from protobuf

  private:
    void set_map(const std::shared_ptr<map::Map>& map_ptr) { map_ptr_ = map_ptr; };
    void set_start(const Eigen::Vector3d& start_vec) { start_vec_ = start_vec; };
    void set_goal(const Eigen::Vector3d& goal_vec) { goal_vec_ = goal_vec; };

    std::shared_ptr<map::Map> map_ptr_;

    Eigen::Vector3d  start_vec_;   // [x, y, theta]
    Eigen::Vector3d  goal_vec_;
    problem::PlanRes plan_res_;
};

}   // namespace solver