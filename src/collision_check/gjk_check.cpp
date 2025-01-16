#include "gjk_check.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"


namespace planning {
namespace collision_check {

GJKCheck::GJKCheck(const kinematic_model::VehicleParam& vehicle_param) {
    vehicle_param_ = vehicle_param;
}

bool GJKCheck::Check(const Polygon2d& polygon1, const Pose& vehicle_pose) {
    const Polygon2d vehicle_polygon = createVehiclePolygon(vehicle_pose);
    // 1. initial support vector
    Vec2d support_vec(1, 0);
    // 2. update simplex

    return false;
}

Polygon2d const GJKCheck::createVehiclePolygon(const Pose& vehicle_pose) {
    const Vec2d vehicle_position(vehicle_pose.x(), vehicle_pose.y());
    Box2d       vehicle_box(vehicle_position, vehicle_pose.theta(), vehicle_param_.length(), vehicle_param_.width());

    return Polygon2d(vehicle_box);
}

const Vec2d GJKCheck::getSupportPoint(const Polygon2d& polygon, const Vec2d& direction) {
    Vec2d support_point;
    polygon.ExtremePoints(direction.Angle(), nullptr, &support_point);

    return support_point;
}
}   // namespace collision_check
}   // namespace planning