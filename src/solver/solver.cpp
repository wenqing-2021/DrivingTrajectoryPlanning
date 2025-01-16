#include "solver/solver.h"
#include "params.pb.h"
#include <memory>
#include <string>
#include <utility>

namespace solver {


Solver::Solver() {
    const std::pair<std::string, std::string> log_info = utils::Logger::LoadYaml("solver");
    utils::Logger::InitialLogger(log_info.first.c_str(), log_info.second.c_str());
};


void Solver::LoadProblem(const problem::PlanProblem& plan_problem, const params::SolverParams& solver_params) {
    // 1. Set cost map
    LOG(INFO) << "Load problem from protobuf...";
    Solver::setMap(std::make_shared<map::Map>(plan_problem, solver_params.map_resolution()));
    // 2. Set start and goal state
    LOG(INFO) << "Set start and goal state...";
    Eigen::Vector3d start_vec(
        plan_problem.init_state().x(), plan_problem.init_state().y(), plan_problem.init_state().theta());
    Solver::setStart(start_vec);
    Eigen::Vector3d goal_vec(
        plan_problem.goal_state().x(), plan_problem.goal_state().y(), plan_problem.goal_state().theta());
    Solver::setGoal(goal_vec);

    // 3. Set plan params
    Solver::setSolverParams(solver_params);

    // 4. Set collision check
    Solver::setCollisionCheck(std::make_shared<collision_check::GJKCheck>(plan_problem.vehicle_param()));
};

const problem::PlanRes& Solver::Run(const problem::SolverInput& solver_input) {
    LOG(INFO) << "Run solver...";
    // 1. load plan problem
    const problem::PlanProblem plan_problem  = solver_input.plan_problem();
    const params::SolverParams solver_params = solver_input.solver_params();
    Solver::LoadProblem(plan_problem, solver_params);

    return plan_res_;
};

}   // namespace solver