#pragma once
#include "collision_check/base_check.h"
#include "planner/hybrid_a_star/hybrid_a_star.h"
#include <string>

namespace planning {

class TrajPlanner {
  public:
    TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                const std::shared_ptr<collision_check::BaseCheck>& collision_checker);
    ~TrajPlanner() = default;

    bool        Process(const Eigen::Vector3d& start_vec,
                        const Eigen::Vector3d& goal_vec);              // main function to generate the trajectory
    inline void GetInitPath(std::vector<Eigen::Vector3d>& init_path,   // get the init path
                            double&                       init_path_length) {
        init_path        = frontend_path_;
        init_path_length = frontend_path_length_;
    };
    inline std::vector<Eigen::Vector3d>& GetDebugNodeList() { return debug_node_list_; };   // get the debug node list
    std::string                          GetName() const { return name_; };             // get the name of the planner
    void                                 SetName(std::string name) { name_ = name; };   // set the name of the planner
  private:
    std::string                            name_ = "TrajPlanner";   // name of the planner
    std::unique_ptr<frontend::HybridAstar> hybrid_astar_ptr_;       // pointer to the hybrid A* planner
    std::vector<Eigen::Vector3d>           frontend_path_;
    std::vector<Eigen::Vector3d>           debug_node_list_;
    double                                 frontend_path_length_{0.0};


};   // class BasePlanner

}   // namespace planning