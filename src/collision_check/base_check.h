#pragma once
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "kinematic_model.pb.h"
#include <memory>
namespace planning {
namespace collision_check {

class BaseCheck {
  public:
    BaseCheck() = default;
    virtual ~BaseCheck(){};

    virtual bool Check(const common::math::Polygon2d& polygon1, const common::math::Pose& vehicle_pose) = 0;

    template<typename T_cls, typename T_param>
    static std::shared_ptr<T_cls> CreateChecker(const T_param& initial_param) {
        return std::make_shared<T_cls>(initial_param);
    };
};
}   // namespace collision_check
}   // namespace planning