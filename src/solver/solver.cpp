#include "solver.h"
#include <string>
#include <utility>

namespace solver {


Solver::Solver() {
    const std::pair<std::string, std::string> log_info = utils::Logger::LoadYaml("solver");
    utils::Logger::InitialLogger(log_info.first.c_str(), log_info.second.c_str());
};


void Solver::load_problem(const problem::PlanProblem& plan_problem) {
    // 1. Load problem from protobuf
    // Set cost map
    LOG(INFO) << "Load problem from protobuf...";
    auto map_ptr = std::make_shared<map::Map>(plan_problem);
    Solver::set_map(map_ptr);
    // Set start and goal state
    LOG(INFO) << "Set start and goal state...";
    Eigen::Vector3d start_vec(
        plan_problem.init_state().x(), plan_problem.init_state().y(), plan_problem.init_state().theta());
    Solver::set_start(start_vec);
    Eigen::Vector3d goal_vec(
        plan_problem.goal_state().x(), plan_problem.goal_state().y(), plan_problem.goal_state().theta());
    Solver::set_goal(goal_vec);
};

const problem::PlanRes& Solver::run(const problem::SolverInput& solver_input) {
    LOG(INFO) << "Run solver...";

    return plan_res_;
};

}   // namespace solver