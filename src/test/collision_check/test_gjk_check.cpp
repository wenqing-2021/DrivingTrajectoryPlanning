#include "gjk_check.h"
#include "kinematic_model.pb.h"
#include "math/polygon2d.h"
#include "math/pose.h"
#include "math/vec2d.h"
#include <gtest/gtest.h>

using namespace planning::collision_check;
using namespace common::math;

class GJKCheckTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Set up vehicle parameters
        vehicle_param_.set_length(4.5);
        vehicle_param_.set_width(2.0);
        vehicle_param_.set_wheel_base(2.5);
        vehicle_param_.set_front_overhang(1.0);
        vehicle_param_.set_rear_overhang(1.0);

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
    bool collision_1 = gjk_check_->Check(polygon1, vehicle_pose);
    bool collision_2 = gjk_check_->Check(polygon2, vehicle_pose);

    // Expect collision
    EXPECT_TRUE(collision_1);
    EXPECT_TRUE(collision_2);
}

TEST_F(GJKCheckTest, vehicle_collision) {
    // 1. define the obstacle polygon
    std::vector<Vec2d> points_obs = {Vec2d(-26.757860973806402, -21.9245275091866),
                                     Vec2d(-12.8250820695946, -16.3677593831667),
                                     Vec2d(-13.544498316309999, -14.5639289410347),
                                     Vec2d(-27.4772772205218, -20.1206970670546)};
    Polygon2d          obs_polygon(points_obs);
    Pose               vehicle_pose(-15.0, -15.0, 0.0);

    bool collision = gjk_check_->Check(obs_polygon, vehicle_pose);

    // Expect collision
    EXPECT_TRUE(collision);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}