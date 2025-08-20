#pragma once

#include "cost_map.pb.h"
#include "math/polygon2d.h"
#include "params.pb.h"
#include "problem.pb.h"
#include <Eigen/Core>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace map {

struct MapDim {
    static constexpr std::uint32_t x = 0, y = 1, nSize = 2;
};

class Map {

  public:
    Map(const problem::PlanProblem& plan_problem, double map_resolution);

    ~Map() = default;

    cost_map::Pos2D ConvertIdxtoXY(std::uint32_t idx_x, std::uint32_t idx_y);
    cost_map::Index ConvertXYtoIdx(double x, double y);

    void                     UpdateESDF();
    void                     UpdateSafeDis(std::uint32_t dim, std::uint32_t idx);
    void                     InitCostMap(const problem::PlanProblem& plan_problem, double map_resolution);
    void                     SetCostMap(const cost_map::CostMap& cost_map) { cost_map_ = cost_map; };
    const cost_map::CostMap& GetCostMap() const { return cost_map_; };
    inline bool              IsInMap(double x, double y) const {
        return (x >= cost_map_.map_info().min_x() && x <= cost_map_.map_info().max_x() &&
                y >= cost_map_.map_info().min_y() && y <= cost_map_.map_info().max_y());
    };
    inline const std::vector<common::math::Polygon2d>& GetObsList() const { return obs_list_; };

  private:
    void initValueList(const std::uint32_t x_size, const std::uint32_t y_size, std::vector<std::vector<double>>& list);
    const double getFValue(std::uint32_t index_x, std::uint32_t index_y) const { return f_list_[index_x][index_y]; };

    void setFValue(std::uint32_t index_x, std::uint32_t index_y, double value) { f_list_[index_x][index_y] = value; };

    cost_map::CostMap                    cost_map_;   // cost map
    std::vector<common::math::Polygon2d> obs_list_;   // obstacle list

    std::vector<std::vector<double>> f_list_;

    // const Dim map_dim{x = 0, y = 1, nSize = 2};
};


}   // namespace map