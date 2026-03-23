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
    Solver::setPose(plan_problem.init_state(), start_vec_);
    Solver::setPose(plan_problem.goal_state(), goal_vec_);

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
        setOptTraj(traj_planner_ptr_->GetOptStates(),
                   traj_planner_ptr_->GetOptControls());   // set the opt traj
        setPreOptTraj(traj_planner_ptr_->GetInitStates(),
                      traj_planner_ptr_->GetInitControls());   // set the pre-opt traj
        LOG(INFO) << "The opt traj has been set...";
        plan_res_.set_solve_success(true);
    } else {
        LOG(WARNING) << "Failed to find the opt path...";
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

void Solver::setOptTraj(const planning::vehicle_model::opt_states&  opt_states,
                        const planning::vehicle_model::opt_control& opt_controls) {
    // 1. check the opt_states and opt_controls
    LOG(INFO) << "Set opt traj...";
    if (opt_states.rows() < 2) {
        LOG(WARNING) << "The opt states size is less than 2";
        return;
    } else if (opt_states.rows() != opt_controls.rows() + 1) {
        LOG(WARNING) << "The opt states size is not equal to the opt controls size + 1";
        return;
    }
    plan_res_.clear_init_traj();
    for (int i = 0; i < opt_states.rows(); ++i) {
        kinematic_model::StateVar   opt_traj_state;
        kinematic_model::ControlVar opt_traj_control;

        const double x     = opt_states(i, 0);
        const double y     = opt_states(i, 1);
        const double theta = opt_states(i, 2);
        const double v     = opt_states(i, 3);
        opt_traj_state.set_x(x);
        opt_traj_state.set_y(y);
        opt_traj_state.set_theta(theta);
        opt_traj_state.set_v(v);
        plan_res_.add_init_traj()->CopyFrom(opt_traj_state);

        if (i < opt_controls.rows()) {
            const double a     = opt_controls(i, 0);
            const double delta = opt_controls(i, 1);
            opt_traj_control.set_accelerate(a);
            opt_traj_control.set_steer_angle(delta);
            plan_res_.add_init_controls()->CopyFrom(opt_traj_control);
        }
    }
};

void Solver::setPreOptTraj(const planning::vehicle_model::opt_states&  init_states,
                           const planning::vehicle_model::opt_control& init_controls) {
    LOG(INFO) << "Set pre-opt traj...";
    if (init_states.rows() < 2) {
        LOG(WARNING) << "The init states size is less than 2";
        return;
    } else if (init_states.rows() != init_controls.rows() + 1) {
        LOG(WARNING) << "The init states size is not equal to the init controls size + 1";
        return;
    }
    plan_res_.clear_pre_opt_traj();
    plan_res_.clear_pre_opt_controls();
    for (int i = 0; i < init_states.rows(); ++i) {
        kinematic_model::StateVar st;
        st.set_x(init_states(i, 0));
        st.set_y(init_states(i, 1));
        st.set_theta(init_states(i, 2));
        st.set_v(init_states(i, 3));
        plan_res_.add_pre_opt_traj()->CopyFrom(st);

        if (i < init_controls.rows()) {
            kinematic_model::ControlVar ct;
            ct.set_accelerate(init_controls(i, 0));
            ct.set_steer_angle(init_controls(i, 1));
            plan_res_.add_pre_opt_controls()->CopyFrom(ct);
        }
    }
};

}   // namespace solver