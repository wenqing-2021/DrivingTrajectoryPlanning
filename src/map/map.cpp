#include "map.h"
#include "logger.h"
#include "problem.pb.h"
#include <cstddef>
#include <limits>
#include <vector>

namespace map {

Map::Map(const problem::PlanProblem& plan_problem, double map_resolution) {
    // Load problem from protobuf
    // 1. set occ map
    InitCostMap(plan_problem, map_resolution);

    // 2. update ESDF
    UpdateESDF();
};

void Map::initValueList(const std::uint32_t x_size, const std::uint32_t y_size,
                        std::vector<std::vector<double>>& list) {
    for (auto& l : list) { l.clear(); }
    list.resize(x_size);
    for (auto& l : list) { l.resize(y_size, 0.0); }
};

void Map::UpdateESDF() {
    // 1. update value on y-axis
    for (std::uint32_t y_idx = 0; y_idx < cost_map_.map_info().y_size(); ++y_idx) { UpdateSafeDis(MapDim::y, y_idx); }
    // 2. update value on x-axis
    for (std::uint32_t x_idx = 0; x_idx < cost_map_.map_info().x_size(); ++x_idx) { UpdateSafeDis(MapDim::x, x_idx); }
    // 3. update cost map
    for (std::uint32_t x_idx = 0; x_idx < cost_map_.map_info().x_size(); ++x_idx) {
        for (std::uint32_t y_idx = 0; y_idx < cost_map_.map_info().y_size(); ++y_idx) {
            cost_map::Point* point = cost_map_.mutable_points(x_idx * cost_map_.map_info().y_size() + y_idx);
            point->set_safe_dis(getFValue(x_idx, y_idx));
        }
    }
};

void Map::UpdateSafeDis(std::uint32_t dim, std::uint32_t idx) {
    auto update_s = [this, &dim, &idx](std::uint32_t& q, int k, const std::vector<double>& v_list) -> double {
        double q_value;
        double v_value;
        if (dim == MapDim::x) {
            q_value = this->getFValue(idx, q);
            v_value = this->getFValue(idx, v_list[k]);
        } else {
            q_value = this->getFValue(q, idx);
            v_value = this->getFValue(v_list[k], idx);
        }
        double s = (q_value + q * q - v_value - v_list[k] * v_list[k]) / (2 * q - 2 * v_list[k] + 1e-5);
        return s;
    };
    // 1. initial
    std::uint32_t end;
    if (dim == MapDim::x) {
        end = cost_map_.map_info().y_size();
    } else {
        end = cost_map_.map_info().x_size();
    }
    std::uint32_t       k = 0;
    std::vector<double> v_list(end, 0.0);
    std::vector<double> z_list(end + 1, 0.0);
    z_list[0] = -std::numeric_limits<double>::max();
    z_list[1] = std::numeric_limits<double>::max();

    // 2. compute lower envolope
    for (std::uint32_t q = 1; q < end; ++q) {
        double s = update_s(q, k, v_list);
        while (s <= z_list[k]) {
            --k;
            s = update_s(q, k, v_list);
        }
        ++k;
        v_list[k]     = q;
        z_list[k]     = s;
        z_list[k + 1] = std::numeric_limits<double>::max();
    }

    // 3. fill safe dis
    k = 0;
    for (std::uint32_t q = 0; q < end; ++q) {
        while (z_list[k + 1] < q) { ++k; }
        if (dim == MapDim::x) {
            double safe_dis = (q - v_list[k]) * (q - v_list[k]) + this->getFValue(idx, v_list[k]);
            setFValue(idx, q, safe_dis);
        } else {
            double safe_dis = (q - v_list[k]) * (q - v_list[k]) + this->getFValue(v_list[k], idx);
            setFValue(q, idx, safe_dis);
        }
    };
};

void Map::InitCostMap(const problem::PlanProblem& plan_problem, double map_resolution) {
    const double map_min_x = plan_problem.map_bound().min_x();
    const double map_min_y = plan_problem.map_bound().min_y();
    const double map_max_x = plan_problem.map_bound().max_x();
    const double map_max_y = plan_problem.map_bound().max_y();
    // 1. discretize the map
    const std::uint32_t x_size   = static_cast<std::uint32_t>((map_max_x - map_min_x) / map_resolution);
    const std::uint32_t y_size   = static_cast<std::uint32_t>((map_max_y - map_min_y) / map_resolution);
    auto                map_info = cost_map_.mutable_map_info();
    map_info->set_max_x(map_max_x);
    map_info->set_max_y(map_max_y);
    map_info->set_min_x(map_min_x);
    map_info->set_min_y(map_min_y);
    map_info->set_resolution(map_resolution);
    map_info->set_x_size(static_cast<uint32_t>(x_size));
    map_info->set_y_size(static_cast<uint32_t>(y_size));
    Map::initValueList(cost_map_.map_info().x_size(), cost_map_.map_info().y_size(), f_list_);

    // 2. create obstacle list
    const int obs_size = plan_problem.obstacle_num();
    LOG(INFO) << "obstacle size: " << obs_size << "start to fill the obstacle list...";
    obs_list_.clear();
    obs_list_.resize(obs_size);
    for (int i = 0; i < obs_size; ++i) {
        const auto&                      obs = plan_problem.obstacle_list(i);
        std::vector<common::math::Vec2d> obs_points;
        obs_points.resize(obs.vertex_num());
        for (int j = 0; j < obs.vertex_num(); ++j) {
            obs_points[j] = common::math::Vec2d(obs.vertex_pts(j).x(), obs.vertex_pts(j).y());
        }
        obs_list_[i] = common::math::Polygon2d(obs_points);
        LOG(INFO) << "obstacle " << i << " is: " << obs_list_[i].DebugString();
    }
    LOG(INFO) << "fill the obstacle list done!";

    // 3. fill the obstacle map
    cost_map_.clear_points();
    for (std::uint32_t x_idx = 0; x_idx < x_size; ++x_idx) {
        for (std::uint32_t y_idx = 0; y_idx < y_size; ++y_idx) {
            auto point = cost_map_.add_points();
            auto pts   = point->mutable_position();
            auto idx   = point->mutable_index();
            pts->set_x(map_min_x + x_idx * map_resolution);
            pts->set_y(map_min_y + y_idx * map_resolution);
            idx->set_idx_x(static_cast<uint32_t>(x_idx));
            idx->set_idx_y(static_cast<uint32_t>(y_idx));
            // check if the point is in the obstacle
            bool   is_occ   = false;
            double safe_dis = std::numeric_limits<double>::max();
            for (const auto& obs : obs_list_) {
                if (obs.IsPointIn(common::math::Vec2d(pts->x(), pts->y()))) {
                    is_occ   = true;
                    safe_dis = 0.0;
                    break;
                }
            }
            point->set_is_occupy(is_occ);
            point->set_safe_dis(safe_dis);
            Map::setFValue(x_idx, y_idx, safe_dis);
        }
    };
};

cost_map::Pos2D Map::ConvertIdxtoXY(std::uint32_t idx_x, std::uint32_t idx_y) {
    cost_map::Pos2D pos;
    pos.set_x(cost_map_.map_info().min_x() + idx_x * cost_map_.map_info().resolution());
    pos.set_y(cost_map_.map_info().min_y() + idx_y * cost_map_.map_info().resolution());
    return pos;
};

cost_map::Index Map::ConvertXYtoIdx(double x, double y) {
    cost_map::Index idx;
    idx.set_idx_x(
        static_cast<std::uint32_t>(std::floor((x - cost_map_.map_info().min_x()) / cost_map_.map_info().resolution())));
    idx.set_idx_y(
        static_cast<std::uint32_t>(std::floor((y - cost_map_.map_info().min_y()) / cost_map_.map_info().resolution())));
    return idx;
};

}   // namespace map