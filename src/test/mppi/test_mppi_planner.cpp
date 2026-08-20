#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "planner/mppi/mppi_planner.h"

namespace planning::mppi {
namespace {

std::vector<VehicleState> StraightReference(std::size_t size, double spacing,
                                            double velocity) {
  std::vector<VehicleState> reference(size);
  for (std::size_t index = 0; index < size; ++index) {
    reference[index].x = spacing * static_cast<double>(index);
    reference[index].velocity = velocity;
  }
  return reference;
}

TEST(MppiPlannerTest, PlansForwardAlongReference) {
  MppiConfig config;
  config.horizon = 20;
  config.num_samples = 256;
  config.num_iterations = 4;
  config.random_seed = 7;
  MppiPlanner planner(config);

  VehicleState initial_state;
  initial_state.velocity = 2.0;
  const std::vector<VehicleState> reference =
      StraightReference(config.horizon + 1, 0.4, 4.0);

  const PlanResult result = planner.Plan(initial_state, reference, {});

  ASSERT_EQ(result.states.size(), config.horizon + 1);
  ASSERT_EQ(result.controls.size(), config.horizon);
  EXPECT_GT(result.states.back().x, initial_state.x);
  EXPECT_NEAR(result.states.back().y, 0.0, 1.0);
  EXPECT_TRUE(std::isfinite(result.cost));
}

TEST(MppiPlannerTest, ResetMakesSeededPlannerDeterministic) {
  MppiConfig config;
  config.horizon = 10;
  config.num_samples = 64;
  config.num_iterations = 2;
  MppiPlanner planner(config);
  const std::vector<VehicleState> reference =
      StraightReference(config.horizon + 1, 0.2, 2.0);

  const PlanResult first = planner.Plan(VehicleState{}, reference, {});
  planner.Reset();
  const PlanResult second = planner.Plan(VehicleState{}, reference, {});

  ASSERT_EQ(first.controls.size(), second.controls.size());
  EXPECT_DOUBLE_EQ(first.cost, second.cost);
  for (std::size_t index = 0; index < first.controls.size(); ++index) {
    EXPECT_DOUBLE_EQ(first.controls[index].steering,
                     second.controls[index].steering);
    EXPECT_DOUBLE_EQ(first.controls[index].acceleration,
                     second.controls[index].acceleration);
  }
}

TEST(MppiPlannerTest, RejectsEmptyReference) {
  MppiPlanner planner(MppiConfig{});
  EXPECT_THROW(planner.Plan(VehicleState{}, {}, {}), std::invalid_argument);
}

}  // namespace
}  // namespace planning::mppi
