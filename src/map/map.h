#pragma once

#include "cost_map.pb.h"
#include "problem.pb.h"
#include <iostream>
#include <string>
#include <vector>

namespace map {

class Map {

  public:
    Map(const problem::PlanProblem& plan_problem);

    ~Map() = default;

    void updateESDF();

    void setCostMap(const cost_map::CostMap& cost_map) { cost_map_ = cost_map; };   // set the cost map

  private:
    cost_map::CostMap cost_map_;   // cost map
};


}   // namespace map