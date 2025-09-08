#pragma once

#include "kinematic_model.pb.h"
#include "logger.h"
#include <Eigen/Core>

namespace planning {
namespace vehicle_model {

using opt_status  = Eigen::MatrixXd;
using opt_control = Eigen::MatrixXd;

struct VehiclePose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

struct VehicleState {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double velocity{0.0};
};

struct ControlSignal {
    double steer_angle{0.0};
    double acceleration{0.0};
};

using sdv_path = std::vector<VehiclePose>;
using sdv_traj = std::vector<VehicleState>;

class KinematicModel {
  public:
    KinematicModel(const kinematic_model::VehicleParam& vehicle_param)
        : vehicle_param_(vehicle_param){};
    ~KinematicModel() = default;
    const kinematic_model::VehicleParam& GetVehicleParam() const { return vehicle_param_; }

  public:
    static std::size_t GetStateSize() {
        static const std::size_t kStateSize = kinematic_model::StateVar::descriptor()->field_count();
        bool not_same_size = static_cast<std::size_t>(kStateSize != sizeof(VehicleState) / sizeof(double));
        if (not_same_size) {
            LOG(ERROR) << "The state size is not same as the VehicleState struct size, please check the proto file!";
        }
        return kStateSize;
    }

    static std::size_t GetControlSize() {
        static const std::size_t kControlSize = kinematic_model::ControlVar::descriptor()->field_count();
        return kControlSize;
    }
    static bool State2Matrix(const std::vector<VehicleState>& state_list, Eigen::MatrixXd* state_vec) {
        if (state_list.empty() || state_vec == nullptr) { return false; }
        const std::size_t kStateSize = GetStateSize();
        state_vec->resize(state_list.size(), kStateSize);
        for (std::size_t i = 0; i < state_list.size(); ++i) {
            (*state_vec)(i, 0) = state_list[i].x;
            (*state_vec)(i, 1) = state_list[i].y;
            (*state_vec)(i, 2) = state_list[i].theta;
            (*state_vec)(i, 3) = state_list[i].velocity;
        }
        return true;
    }
    static bool Pose2Matrix(const sdv_path& pose_list, Eigen::MatrixXd* pose_vec) {
        if (pose_list.empty() || pose_vec == nullptr) { return false; }
        const std::size_t kPoseSize = 3;
        pose_vec->resize(pose_list.size(), kPoseSize);
        for (std::size_t i = 0; i < pose_list.size(); ++i) {
            (*pose_vec)(i, 0) = pose_list[i].x;
            (*pose_vec)(i, 1) = pose_list[i].y;
            (*pose_vec)(i, 2) = pose_list[i].theta;
        }
        return true;
    }
    static bool Control2Matrix(const std::vector<ControlSignal>& control_list, Eigen::MatrixXd* control_vec) {
        if (control_list.empty() || control_vec == nullptr) { return false; }
        const std::size_t kControlSize = GetControlSize();
        control_vec->resize(control_list.size(), kControlSize);
        for (std::size_t i = 0; i < control_list.size(); ++i) {
            (*control_vec)(i, 0) = control_list[i].steer_angle;
            (*control_vec)(i, 1) = control_list[i].acceleration;
        }
        return true;
    }

    template<typename StateType>
    static bool getNextState(const StateType& current_state, const ControlSignal& control_signal,
                             const kinematic_model::VehicleParam& vehicle_param, const double delta_time,
                             StateType* next_state) {
        if (next_state == nullptr) { return false; }
        // 1. get the current state
        double x         = current_state.x;
        double y         = current_state.y;
        double theta     = current_state.theta;
        double velocity  = current_state.velocity;
        double steer_ang = control_signal.steer_angle;
        double accel     = control_signal.acceleration;
        // 2. compute the next state using bicycle model
        double beta          = std::atan(0.5 * std::tan(steer_ang));   // slip angle
        next_state->x        = x + velocity * std::cos(theta + beta) * delta_time;
        next_state->y        = y + velocity * std::sin(theta + beta) * delta_time;
        next_state->theta    = theta + (velocity / vehicle_param.wheel_base()) * std::sin(beta) * delta_time;
        next_state->velocity = velocity + accel * delta_time;
        // 3. normalize the theta to [-pi, pi]
        if (next_state->theta > M_PI) {
            next_state->theta -= 2 * M_PI;
        } else if (next_state->theta < -M_PI) {
            next_state->theta += 2 * M_PI;
        }
        return true;
    }


  private:
    kinematic_model::VehicleParam vehicle_param_;
};

}   // namespace vehicle_model
}   // namespace planning