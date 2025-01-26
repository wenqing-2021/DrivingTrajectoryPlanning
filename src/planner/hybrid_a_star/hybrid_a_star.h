#pragma once

#include "collision_check/base_check.h"
#include "logger/logger.h"
#include "map/map.h"
#include "params.pb.h"
#include "planner/hybrid_a_star/rs_path.h"
#include <Eigen/Core>
#include <memory>
#include <queue>
#include <vector>


namespace planning {
namespace frontend {

class HybridAstar {
  public:
    HybridAstar(const params::HybridAStarParams& hybrid_params, const std::shared_ptr<map::Map>& map_ptr);
    ~HybridAstar() = default;

    bool Plan(const Eigen::Vector3d& start_vec, const Eigen::Vector3d& goal_vec);

  private:
    struct Node {
      public:
        explicit Node(const double& x, const double& y, const double& theta, const double& steer_angle,
                      const bool& is_forward, const std::shared_ptr<Node>& parent_ptr)
            : x_(x)
            , y_(y)
            , theta_(theta)
            , is_forward_(is_forward)
            , steer_angle_(steer_angle)
            , parent_ptr_(parent_ptr){};

        void updateValue(const double& g_value, const double& h_value) {
            g_value_ = g_value;
            h_value_ = h_value;
            f_value_ = g_value_ + h_value_;
        };
        void updateParent(const std::shared_ptr<Node>& parent_ptr) { parent_ptr_ = parent_ptr; };

      public:
        double                x_;
        double                y_;
        double                theta_;
        double                steer_angle_;
        std::uint32_t         index_x_;
        std::uint32_t         index_y_;
        bool                  is_closed_;
        bool                  is_open_;
        bool                  is_forward_;
        double                g_value_;
        double                h_value_;
        double                f_value_;
        std::shared_ptr<Node> parent_ptr_;
        std::shared_ptr<Node> child_ptr_;
    };
    struct CompareNode {
        bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
            return lhs->f_value_ < rhs->f_value_;   // Greater f_value will have higher priority
        }
    };

  private:
    void setParams(const params::HybridAStarParams& hybrid_params) { hybrid_params_ = hybrid_params; };
    void setMap(const std::shared_ptr<map::Map>& map_ptr) { map_ptr_ = map_ptr; };
    void setRSPath(std::unique_ptr<RSPath>& rs_path_ptr) { rs_path_ptr_ = std::move(rs_path_ptr); };

  private:
    bool isReach(const std::shared_ptr<Node>& node_ptr);
    bool isCollision(const std::shared_ptr<Node>& node_ptr);
    bool isClosedList(const std::shared_ptr<Node>& node_ptr);
    bool isOpenList(const std::shared_ptr<Node>& node_ptr);

    double calcHValue(const std::shared_ptr<Node>& child_node_ptr);
    double calcGValue(const std::shared_ptr<Node>& curr_node_ptr, const std::shared_ptr<Node>& child_node_ptr);
    void   ExpandNode(const std::shared_ptr<Node>& node_ptr);
    void   initNode();

  private:
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list_;

    static constexpr double            kEpsilon = 1e-6;
    params::HybridAStarParams          hybrid_params_;
    std::vector<std::shared_ptr<Node>> close_list_;
    std::shared_ptr<map::Map>          map_ptr_;
    std::shared_ptr<Node>              selected_node_;
    std::unique_ptr<RSPath>            rs_path_ptr_;
    Eigen::Vector3d                    start_vec_;
    Eigen::Vector3d                    goal_vec_;

};   // Class definition

}   // namespace frontend
}   // namespace planning
