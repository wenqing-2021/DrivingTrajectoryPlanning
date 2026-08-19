#ifndef SRC_PLANNER_MPPI_MPPI_PLANNER_H_
#define SRC_PLANNER_MPPI_MPPI_PLANNER_H_

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace planning::mppi {

struct VehicleState {
    double x        = 0.0;
    double y        = 0.0;
    double yaw      = 0.0;
    double velocity = 0.0;
};

struct Control {
    double steering     = 0.0;
    double acceleration = 0.0;
};

struct CircularObstacle {
    double x      = 0.0;
    double y      = 0.0;
    double radius = 0.0;
};

struct MppiConfig {
    std::size_t horizon            = 30;
    std::size_t num_samples        = 512;
    std::size_t num_iterations     = 4;
    double      time_step          = 0.1;
    double      temperature        = 1.0;
    double      wheelbase          = 2.7;
    double      min_steering       = -0.55;
    double      max_steering       = 0.55;
    double      min_acceleration   = -4.0;
    double      max_acceleration   = 2.5;
    double      min_velocity       = 0.0;
    double      max_velocity       = 15.0;
    double      steering_noise     = 0.2;
    double      acceleration_noise = 1.0;
    // Correlation coefficient in [0, 1) for temporally smoothing the sampled
    // noise. Higher values produce smoother sampled rollouts; 0 keeps the
    // classic per-step white-noise sampling.
    double noise_smoothing = 0.95;
    double position_weight = 3.0;
    double heading_weight  = 1.0;
    double velocity_weight = 2.0;
    double control_weight  = 0.08;
    // Penalty on the squared change of steering/acceleration between
    // consecutive controls. Directly discourages jagged, unsmooth controls.
    double control_rate_weight = 8.0;
    double terminal_weight     = 8.0;
    double obstacle_weight     = 35.0;
    // Decay length (meters) of the exponential obstacle potential. Larger
    // values make the repulsion act over a longer range with a gentler
    // gradient, so the vehicle commits to a smooth detour early instead of
    // reacting sharply at close range.
    double obstacle_softening = 2.5;
    double collision_cost     = 1.0e5;
    double vehicle_radius     = 1.4;
    // Lateral road boundaries (meters, relative to the reference heading,
    // positive to the left) and the penalty weight for leaving the corridor.
    // These keep the vehicle on the drivable surface while it detours around
    // obstacles. Defaults match the two-lane demo corridor around the
    // reference centerline.
    double        road_left_limit      = 5.0;
    double        road_right_limit     = -2.0;
    double        road_boundary_weight = 50.0;
    std::uint32_t random_seed          = 42;
};

struct PlanResult {
    std::vector<VehicleState> states;
    std::vector<Control>      controls;
    double                    cost = 0.0;
};

// Sampling-based, receding-horizon trajectory planner using the MPPI update
// law.
class MppiPlanner {
  public:
    explicit MppiPlanner(MppiConfig config);

    PlanResult Plan(const VehicleState& initial_state, const std::vector<VehicleState>& reference,
                    const std::vector<CircularObstacle>& obstacles);

    void              Reset();
    const MppiConfig& config() const { return config_; }

  private:
    static double NormalizeAngle(double angle);
    static double Clamp(double value, double lower, double upper);

    VehicleState              Propagate(const VehicleState& state, const Control& control) const;
    std::vector<VehicleState> Rollout(const VehicleState& initial_state, const std::vector<Control>& controls) const;
    double                    ComputeCost(const std::vector<VehicleState>& states, const std::vector<Control>& controls,
                                          const std::vector<VehicleState>&     reference,
                                          const std::vector<CircularObstacle>& obstacles) const;
    void                      ValidateInputs(const std::vector<VehicleState>& reference) const;
    void                      ShiftControlSequence();

    MppiConfig           config_;
    std::vector<Control> nominal_controls_;
    // Control applied at the previous receding-horizon step; used as the
    // continuity anchor for the first control-rate penalty of the next plan.
    Control      last_applied_control_{};
    std::mt19937 random_generator_;
};

}   // namespace planning::mppi

#endif   // SRC_PLANNER_MPPI_MPPI_PLANNER_H_
