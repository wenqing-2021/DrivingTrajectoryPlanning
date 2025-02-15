#pragma once
#include "base_check.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "kinematic_model.pb.h"
#include <vector>

namespace planning {
namespace collision_check {

using namespace common::math;

class GJKCheck : public BaseCheck {
  public:
    GJKCheck(const kinematic_model::VehicleParam& vehicle_param);
    ~GJKCheck() override{};

    bool            Check(const Polygon2d& polygon1, const Pose& vehicle_pose) override;
    Polygon2d const CreateVehiclePolygon(const Pose& vehicle_pose);
    const bool      CheckInTriangle(const Vec2d& A, const Vec2d& B, const Vec2d& C, const Vec2d& D);

  private:
    void        initialSuportVec();
    const Vec2d getSupportPoint(const Polygon2d& polygon1, const Polygon2d& polygon2);
    void        initialSimplex(const Polygon2d& polygon1, const Polygon2d& polygon2);
    void        updateSimplex(const Vec2d& support_point, std::uint32_t idx);
    void        updateVehiclePolygon(const Pose& vehicle_pose);
    bool        checkSupportPoint(const Vec2d& support_point);
    bool        nearestSimplex(std::uint32_t idx);

    const std::pair<Vec2d, Vec2d> getNearestEdge();
    bool                          find_same_point_{false};
    kinematic_model::VehicleParam vehicle_param_;
    std::vector<Vec2d>            simplex_;   // only onsider 2D case, so the max size of simplex is 3 (traiangle)
    Vec2d                         support_vector_;
    const std::uint32_t           kMaxIterNum = 50;
};


}   // namespace collision_check
}   // namespace planning