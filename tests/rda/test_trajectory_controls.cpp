#include "trajectory_planner.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

int main() {
    constexpr double wheelbase = 2.5;
    const std::vector<double> times{0.0, 0.1, 0.3, 0.45, 0.7, 0.8, 1.0};
    const std::vector<double> speeds{0.0, 1.0, 0.0, -1.0, -2.0, 0.0, 0.0};
    const std::vector<double> steering{0.0, 0.7, 0.7, -0.7, 0.4, 0.4};
    Eigen::MatrixXd states(7, 4);
    states.row(0) << 0.0, 0.0, 3.13, speeds[0];
    for (int i = 0; i < 6; ++i) {
        const double dt = times[i + 1] - times[i];
        const double yaw = states(i, 2);
        const double next_yaw = yaw + speeds[i] / wheelbase * std::tan(steering[i]) * dt;
        states.row(i + 1) << states(i, 0) + speeds[i] * std::cos(yaw) * dt,
                            states(i, 1) + speeds[i] * std::sin(yaw) * dt,
                            std::atan2(std::sin(next_yaw), std::cos(next_yaw)), speeds[i + 1];
    }
    const auto controls = planning::TrajPlanner::ReconstructTrajectoryControls(states, times, wheelbase);
    for (int i = 0; i < 6; ++i) {
        const double dt = times[i + 1] - times[i];
        if (std::abs(controls(i, 1) - steering[i]) > 1e-10 ||
            std::abs(speeds[i] + controls(i, 0) * dt - speeds[i + 1]) > 1e-10) {
            std::cerr << "Forward/reverse control round trip failed at " << i << '\n';
            return 1;
        }
    }
    // A stationary first segment with tiny heading noise must not imply full lock.
    states(1, 2) += 1e-5;
    if (planning::TrajPlanner::ReconstructTrajectoryControls(states, times, wheelbase)(0, 1) != 0.0) {
        return 2;
    }
    auto invalid_times = times;
    invalid_times[2] = invalid_times[1];
    try {
        planning::TrajPlanner::ReconstructTrajectoryControls(states, invalid_times, wheelbase);
        return 3;
    } catch (const std::invalid_argument&) {
    }
    std::cout << "Control reconstruction: reverse, stops, heading wrap, nonuniform timing passed\n";
    return 0;
}
