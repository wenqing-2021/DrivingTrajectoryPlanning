#pragma once
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
namespace planning {
namespace collision_check {

class BaseCheck {
  public:
    BaseCheck();
    virtual ~BaseCheck() = 0;

    virtual bool Check(const common::math::Polygon2d& polygon1, const common::math::Pose& vehicle_pose) = 0;
};
}   // namespace collision_check
}   // namespace planning