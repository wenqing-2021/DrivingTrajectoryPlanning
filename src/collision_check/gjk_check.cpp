#include "gjk_check.h"
#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "logger/logger.h"
#include <vector>


namespace planning {
namespace collision_check {

GJKCheck::GJKCheck(const kinematic_model::VehicleParam& vehicle_param) {
    vehicle_param_ = vehicle_param;
    initialSuportVec();
}

void GJKCheck::initialSuportVec() {
    // 1. initial simplex
    simplex_.clear();
    simplex_.resize(3, Vec2d(0.0, 0.0));
    support_vector_  = Vec2d(1.0, 0.0);   // initial with x-axis
    find_same_point_ = false;
}

bool GJKCheck::Check(const Polygon2d& polygon1, const Pose& vehicle_pose) {
    const Polygon2d vehicle_polygon = CreateVehiclePolygon(vehicle_pose);
    // 1. initial simplex
    initialSuportVec();
    initialSimplex(polygon1, vehicle_polygon);   // simplex size is zero
    support_vector_ = -1.0 * simplex_.front();   // update support vector
    // 2. check collision
    for (std::uint32_t idx = 1; idx <= kMaxIterNum; ++idx) {
        Vec2d support_point = getSupportPoint(polygon1, vehicle_polygon);
        if (checkSupportPoint(support_point)) {
            return false;
        }   // The dot of support vector and support point is less than 0
        updateSimplex(support_point, idx);
        if (find_same_point_) { return false; }     // find the same point in simplex
        if (nearestSimplex(idx)) { return true; }   // Triangle includes the origin
    }

    LOG(ERROR) << "GJKCheck::Check: GJK algorithm failed to find collision.";

    return true;
}

const bool GJKCheck::CheckInTriangle(const Vec2d& A, const Vec2d& B, const Vec2d& C, const Vec2d& D) {
    const double area_ABC = GetTriangleAera(A, B, C);
    const double area_ABD = GetTriangleAera(A, B, D);
    const double area_ACD = GetTriangleAera(A, C, D);
    const double area_BCD = GetTriangleAera(B, C, D);
    return std::abs(area_ABC - (area_ABD + area_ACD + area_BCD)) < 1e-6;
}

bool GJKCheck::nearestSimplex(std::uint32_t idx) {
    auto updateSupportVec = [](const Vec2d& A, const Vec2d& B) -> Vec2d {
        // 1. get the last two points
        Vec2d AB = B - A;
        Vec2d AO = -1.0 * A;
        // 2.  compute the support vector
        std::vector<double> vec_AB        = {AB.x(), AB.y(), 0.0};
        std::vector<double> vec_AO        = {AO.x(), AO.y(), 0.0};
        std::vector<double> vec_normal    = Product3D(vec_AB, vec_AO);
        std::vector<double> vec_AB_normal = Product3D(vec_normal, vec_AB);
        // 3. update the support vector
        return Vec2d(vec_AB_normal[0], vec_AB_normal[1]);
    };
    if (idx == 1) {
        support_vector_ = updateSupportVec(simplex_.at(0), simplex_.at(1));
        if (support_vector_.Length() < 1e-6) {
            Vec2d AB        = simplex_.at(1) - simplex_.at(0);
            support_vector_ = Vec2d(AB.y(), -AB.x());
        }
        return false;
    } else {
        // 1. check the triangle is including the origin or not
        const Vec2d A = simplex_.at(0);
        const Vec2d B = simplex_.at(1);
        const Vec2d C = simplex_.at(2);
        const Vec2d O = Vec2d(0.0, 0.0);
        if (CheckInTriangle(A, B, C, O)) {
            return true;
        } else {
            // 2. find the nearest edge to the origin and update the support vector
            const std::pair<Vec2d, Vec2d> nearest_edge = getNearestEdge();

            support_vector_ = updateSupportVec(nearest_edge.first, nearest_edge.second);
            return false;
        }
    }
}

bool GJKCheck::checkSupportPoint(const Vec2d& support_point) {
    return support_vector_.InnerProd(support_point) <= 0.0;
}

void GJKCheck::initialSimplex(const Polygon2d& polygon1, const Polygon2d& polygon2) {
    Vec2d simplex_0 = getSupportPoint(polygon1, polygon2);
    simplex_[0]     = simplex_0;
}

Polygon2d const GJKCheck::CreateVehiclePolygon(const Pose& vehicle_pose) {
    double vehicle_pose_mid_x = vehicle_pose.x() + (vehicle_param_.length() / 2 - vehicle_param_.rear_overhang()) *
                                                       std::cos(vehicle_pose.theta());
    double vehicle_pose_mid_y = vehicle_pose.y() + (vehicle_param_.length() / 2 - vehicle_param_.rear_overhang()) *
                                                       std::sin(vehicle_pose.theta());
    Vec2d vehicle_pose_mid(vehicle_pose_mid_x, vehicle_pose_mid_y);
    Box2d vehicle_box(vehicle_pose_mid, vehicle_pose.theta(), vehicle_param_.length(), vehicle_param_.width());

    return Polygon2d(vehicle_box);
}

const Vec2d GJKCheck::getSupportPoint(const Polygon2d& polygon1, const Polygon2d& polygon2) {
    const Vec2d rever_support_vector = support_vector_ * -1.0;
    Vec2d       support_point_1, nearest_pts_1;
    Vec2d       support_point_2, nearest_pts_2;
    polygon1.ExtremePoints(support_vector_.Angle(), &nearest_pts_1, &support_point_1);
    polygon2.ExtremePoints(rever_support_vector.Angle(), &nearest_pts_2, &support_point_2);

    return support_point_1 - support_point_2;
}

const std::pair<Vec2d, Vec2d> GJKCheck::getNearestEdge() {
    // 1. find the nearest edge to the origin from the simplex
    const LineSegment2d AB(simplex_.at(0), simplex_.at(1));
    const LineSegment2d BC(simplex_.at(1), simplex_.at(2));
    const LineSegment2d CA(simplex_.at(2), simplex_.at(0));
    const Vec2d         O(0.0, 0.0);
    const double        dist_AB = std::abs(AB.ProductOntoUnit(O));
    const double        dist_BC = std::abs(BC.ProductOntoUnit(O));
    const double        dist_CA = std::abs(CA.ProductOntoUnit(O));
    if (dist_AB < dist_BC && dist_AB < dist_CA) {
        // 2. put the furthest point in the front
        auto temp      = simplex_.at(2);
        simplex_.at(2) = simplex_.at(0);
        simplex_.at(0) = temp;
        return std::make_pair(simplex_.at(2), simplex_.at(1));
    } else if (dist_BC < dist_AB && dist_BC < dist_CA) {
        return std::make_pair(simplex_.at(1), simplex_.at(2));
    } else {
        auto temp      = simplex_.at(1);
        simplex_.at(1) = simplex_.at(0);
        simplex_.at(0) = temp;
        return std::make_pair(simplex_.at(1), simplex_.at(2));
    }
}

void GJKCheck::updateSimplex(const Vec2d& support_point, std::uint32_t idx) {
    if (idx < 3) {
        simplex_[idx] = support_point;
        return;
    } else {
        // 0. check the duplicate point
        for (const Vec2d& point : simplex_) {
            if (point == support_point) {
                find_same_point_ = true;
                return;
            }
        }
        // 1. pop the first point in simplex_
        simplex_.erase(simplex_.begin());
        // 2. push the new support point
        simplex_.push_back(support_point);
    }
}

}   // namespace collision_check
}   // namespace planning