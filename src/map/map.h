#pragma once

#include "common/math/polygon2d.h"
#include "cost_map.pb.h"
#include "params.pb.h"
#include "problem.pb.h"
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
    Map(const problem::PlanProblem& plan_problem, const double& map_resolution);

    ~Map() = default;

    cost_map::Pos2D ConvertIdxtoXY(const std::uint32_t& idx_x, const std::uint32_t& idx_y);
    cost_map::Index ConvertXYtoIdx(const double& x, const double& y);

    void                     UpdateESDF();
    void                     UpdateSafeDis(const std::uint32_t& dim, const std::uint32_t& idx);
    void                     InitCostMap(const problem::PlanProblem& plan_problem, const double& map_resolution);
    void                     SetCostMap(const cost_map::CostMap& cost_map) { cost_map_ = cost_map; };
    const cost_map::CostMap& GetCostMap() const { return cost_map_; };

  private:
    void initValueList(const std::uint32_t x_size, const std::uint32_t y_size, std::vector<std::vector<double>>& list);
    const double getFValue(const std::uint32_t& index_x, const std::uint32_t& index_y) const {
        return f_list_[index_x][index_y];
    };

    void setFValue(const std::uint32_t& index_x, const std::uint32_t& index_y, const double& value) {
        f_list_[index_x][index_y] = value;
    };

    cost_map::CostMap                    cost_map_;   // cost map
    std::vector<common::math::Polygon2d> obs_list_;   // obstacle list

    std::vector<std::vector<double>> v_list_{MapDim::nSize};   // dim: [x, y]
    std::vector<std::vector<double>> z_list_{MapDim::nSize};
    std::vector<std::vector<double>> f_list_{MapDim::nSize};

    // const Dim map_dim{x = 0, y = 1, nSize = 2};
};


}   // namespace map