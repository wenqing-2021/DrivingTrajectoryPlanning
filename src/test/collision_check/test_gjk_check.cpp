#include "collision_check/gjk_check.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "kinematic_model.pb.h"
#include <gtest/gtest.h>

using namespace planning::collision_check;
using namespace common::math;

class GJKCheckTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Set up vehicle parameters
        vehicle_param_.set_length(4.5);
        vehicle_param_.set_width(2.0);

        // Initialize GJKCheck object
        gjk_check_ = std::make_unique<GJKCheck>(vehicle_param_);
    }

    kinematic_model::VehicleParam vehicle_param_;
    std::unique_ptr<GJKCheck>     gjk_check_;
};

TEST_F(GJKCheckTest, NoCollision) {
    // Define two non-colliding polygons
    std::vector<Vec2d> points1 = {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), Vec2d(1.0, 1.0), Vec2d(0.0, 1.0)};
    Polygon2d          polygon1(points1);

    Pose vehicle_pose(15.0, 15.0, 0.0);

    // Check for collision
    bool collision = gjk_check_->Check(polygon1, vehicle_pose);

    // Expect no collision
    EXPECT_FALSE(collision);
}

TEST_F(GJKCheckTest, Collision) {
    // Define two colliding polygons
    std::vector<Vec2d> points1 = {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0), Vec2d(1.0, 1.0), Vec2d(0.0, 1.0)};
    std::vector<Vec2d> points2 = {Vec2d(0.5, 0.5), Vec2d(1.5, 0.5), Vec2d(1.5, 1.5), Vec2d(0.5, 1.5)};
    Polygon2d          polygon1(points1);
    Polygon2d          polygon2(points2);

    Pose vehicle_pose(0.5, 0.0, 0.0);

    // Check for collision
    bool collision = gjk_check_->Check(polygon1, vehicle_pose);

    // Expect collision
    EXPECT_TRUE(collision);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}