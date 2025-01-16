#pragma once
#include "base_check.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "kinematic_model.pb.h"

namespace planning {
namespace collision_check {

using namespace common::math;

class GJKCheck : public BaseCheck {
  public:
    GJKCheck(const kinematic_model::VehicleParam& vehicle_param);
    ~GJKCheck() override = default;

    bool            Check(const Polygon2d& polygon1, const Pose& vehicle_pose) override;
    Polygon2d const createVehiclePolygon(const Pose& vehicle_pose);

  private:
    const Vec2d getSupportPoint(const Polygon2d& polygon, const Vec2d& direction);
    void        updateSimplex(const Vec2d& point);
    void        updateVehiclePolygon(const Pose& vehicle_pose);

    kinematic_model::VehicleParam vehicle_param_;
};


}   // namespace collision_check
}   // namespace planning