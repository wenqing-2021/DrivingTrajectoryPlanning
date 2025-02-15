#include "solver/solver.h"
#include "collision_check/gjk_check.h"
#include "kinematic_model.pb.h"
#include "logger/logger.h"
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
    LOG(INFO) << "Set collision check...";
    Solver::setCollisionCheck(
        collision_check::BaseCheck::CreateChecker<collision_check::GJKCheck>(plan_problem.vehicle_param()));

    // 5. Set trajectory planner
    LOG(INFO) << "Set trajectory planner...";
    traj_planner_ptr_ = std::make_unique<planning::TrajPlanner>(solver_params, map_ptr_, collision_check_ptr_);
};

const problem::PlanRes& Solver::Run(const problem::SolverInput& solver_input) {
    LOG(INFO) << "Run solver...";
    // 1. load plan problem
    const problem::PlanProblem plan_problem  = solver_input.plan_problem();
    const params::SolverParams solver_params = solver_input.solver_params();
    LOG(INFO) << "Load problem from protobuf...";
    Solver::LoadProblem(plan_problem, solver_params);
    plan_res_.set_solve_success(false);
    // 2. process
    LOG(INFO) << "Process...";
    if (traj_planner_ptr_->Process(start_vec_, goal_vec_)) {
        std::vector<Eigen::Vector3d> init_path;
        double                       init_path_length;
        traj_planner_ptr_->GetInitPath(init_path, init_path_length);
        LOG(INFO) << "The init path is found...";
        plan_res_.clear_init_path();
        for (const auto& point : init_path) {
            kinematic_model::StateVar init_path_state;
            init_path_state.set_x(point.x());
            init_path_state.set_y(point.y());
            init_path_state.set_theta(point.z());
            plan_res_.add_init_path()->CopyFrom(init_path_state);
        }
        plan_res_.set_init_path_length(init_path_length);
        plan_res_.set_solve_success(true);
    } else {
        LOG(WARNING) << "Failed to find the init path...";
        const auto& debug_node_list = traj_planner_ptr_->GetDebugNodeList();
        for (const auto& point : debug_node_list) {
            kinematic_model::StateVar debug_node;
            debug_node.set_x(point.x());
            debug_node.set_y(point.y());
            debug_node.set_theta(point.z());
            plan_res_.add_init_path()->CopyFrom(debug_node);
        }
        plan_res_.set_solve_success(false);
    }

    return plan_res_;
};

}   // namespace solver