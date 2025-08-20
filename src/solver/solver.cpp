#include "solver/solver.h"
#include "collision_check/gjk_check.h"
#include "kinematic_model.pb.h"
#include "logger.h"
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
    traj_planner_ptr_ = std::make_unique<planning::TrajPlanner>(
        solver_params, map_ptr_, collision_check_ptr_, plan_problem.vehicle_param());
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
        setInitTraj(traj_planner_ptr_->GetInitStates(),
                    traj_planner_ptr_->GetInitControls());   // set the init traj
        LOG(INFO) << "The init traj has been set...";
        plan_res_.set_solve_success(true);
    } else {
        LOG(WARNING) << "Failed to find the init path...";
        const auto& debug_node_list = traj_planner_ptr_->GetDebugNodeList();
        plan_res_.clear_init_traj();
        for (const auto& point : debug_node_list) {
            kinematic_model::StateVar debug_node;
            debug_node.set_x(point.x());
            debug_node.set_y(point.y());
            debug_node.set_theta(point.z());
            debug_node.set_v(0.0);
            plan_res_.add_init_traj()->CopyFrom(debug_node);
        }
        plan_res_.set_solve_success(false);
    }

    return plan_res_;
};

void Solver::setInitTraj(const planning::vehicle_model::opt_status&  init_states,
                         const planning::vehicle_model::opt_control& init_controls) {
    // 1. check the init_states and init_controls
    LOG(INFO) << "Set init traj...";
    if (init_states.rows() < 2) {
        LOG(WARNING) << "The init states size is less than 2";
        return;
    } else if (init_states.rows() != init_controls.rows() + 1) {
        LOG(WARNING) << "The init states size is not equal to the init controls size + 1";
        return;
    }
    plan_res_.clear_init_traj();
    for (int i = 0; i < init_states.rows(); ++i) {
        kinematic_model::StateVar   init_traj_state;
        kinematic_model::ControlVar init_traj_control;

        const double x     = init_states(i, 0);
        const double y     = init_states(i, 1);
        const double theta = init_states(i, 2);
        const double v     = init_states(i, 3);
        init_traj_state.set_x(x);
        init_traj_state.set_y(y);
        init_traj_state.set_theta(theta);
        init_traj_state.set_v(v);
        plan_res_.add_init_traj()->CopyFrom(init_traj_state);

        if (i < init_controls.rows()) {
            const double a     = init_controls(i, 0);
            const double delta = init_controls(i, 1);
            init_traj_control.set_accelerate(a);
            init_traj_control.set_steer_angle(delta);
            plan_res_.add_init_controls()->CopyFrom(init_traj_control);
        }
    }
    LOG(INFO) << "Have set init traj...";
};

}   // namespace solver