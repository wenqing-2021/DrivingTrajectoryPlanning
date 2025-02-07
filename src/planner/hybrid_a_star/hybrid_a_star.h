#pragma once

#include "collision_check/base_check.h"
#include "common/math/math_utils.h"
#include "logger/logger.h"
#include "map/map.h"
#include "params.pb.h"
#include "planner/hybrid_a_star/rs_path.h"
#include <Eigen/Core>
#include <iomanip>
#include <memory>
#include <queue>
#include <vector>


namespace planning {
namespace frontend {

class HybridAstar {
  public:
    HybridAstar(const params::HybridAStarParams& hybrid_params, const std::shared_ptr<map::Map>& map_ptr,
                const std::shared_ptr<collision_check::BaseCheck>& collison_checker);
    ~HybridAstar() = default;

    bool Plan(const Eigen::Vector3d& start_vec, const Eigen::Vector3d& goal_vec);

  private:
    enum NODE_STATUS
    {
        OPEN,
        CLOSE,
        NONE
    };
    struct Node {
      public:
        explicit Node(double x, double y, double theta, double steer_angle, bool is_forward,
                      const std::shared_ptr<Node>& parent_ptr)
            : x_(x)
            , y_(y)
            , theta_(theta)
            , is_forward_(is_forward)
            , steer_angle_(steer_angle)
            , parent_ptr_(parent_ptr) {
            pose_ << x, y, theta;
        };

        void setValue(double g_value, double h_value) {
            g_value_ = g_value;
            h_value_ = h_value;
            f_value_ = g_value_ + h_value_;
        };

      public:
        double                x_;
        double                y_;
        double                theta_;
        double                steer_angle_;
        std::uint32_t         index_x_;
        std::uint32_t         index_y_;
        NODE_STATUS           status_;   // open or closed
        bool                  is_forward_;
        double                g_value_;
        double                h_value_;
        double                f_value_;
        std::shared_ptr<Node> parent_ptr_;
        std::shared_ptr<Node> child_ptr_;
        Eigen::Vector3d       pose_;
    };
    struct CompareNode {
        bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
            return lhs->f_value_ < rhs->f_value_;   // Greater f_value will have higher priority
        }
    };

  private:
    inline void setParams(const params::HybridAStarParams& hybrid_params) { hybrid_params_ = hybrid_params; };
    inline void setMap(const std::shared_ptr<map::Map>& map_ptr) { map_ptr_ = map_ptr; };
    inline void setRSPath(std::unique_ptr<RSPath> rs_path_ptr) { rs_path_ptr_ = std::move(rs_path_ptr); };
    inline void setCollisionChecker(const std::shared_ptr<collision_check::BaseCheck>& collision_checker) {
        collision_checker_ = collision_checker;
    };
    inline void convertToIndex(double x, double y, double theta, std::uint32_t& index_x, std::uint32_t& index_y,
                               std::uint32_t& index_theta) {
        index_x = static_cast<std::uint32_t>(
            std::floor((x - map_ptr_->GetCostMap().map_info().min_x()) / hybrid_params_.node_resolution_x()));
        index_y = static_cast<std::uint32_t>(
            std::floor((y - map_ptr_->GetCostMap().map_info().min_y()) / hybrid_params_.node_resolution_y()));
        index_theta = static_cast<std::uint32_t>(
            std::floor(common::math::NormalizeAngle(theta) / hybrid_params_.node_resolution_theta()));
    };

  private:
    bool isReach(const std::shared_ptr<Node>& node_ptr);
    bool tryRSPath(const std::shared_ptr<Node>& node_ptr);
    bool isCollide(double x, double y, double theta);

    double calcHValue(const std::shared_ptr<Node>& child_node_ptr);
    double calcGValue(const std::shared_ptr<Node>& curr_node_ptr, const std::shared_ptr<Node>& child_node_ptr);
    void   expandNode(const std::shared_ptr<Node>& node_ptr);
    bool   getChildNode(const double expand_s, const double steer_angle, double& child_x, double& child_y,
                        double& child_theta, const std::shared_ptr<Node>& node_ptr);
    void   finishPath();
    void   initNode();

  private:
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> open_list_;
    std::vector<std::vector<std::vector<std::shared_ptr<Node>>>>                                node_map_;

    static constexpr double                     kEpsilon = 1e-6;
    params::HybridAStarParams                   hybrid_params_;
    std::shared_ptr<map::Map>                   map_ptr_;
    std::shared_ptr<collision_check::BaseCheck> collision_checker_;
    std::shared_ptr<Node>                       selected_node_;
    std::unique_ptr<RSPath>                     rs_path_ptr_;
    Eigen::Vector3d                             start_vec_, goal_vec_;

    // result
    std::vector<Eigen::Vector3d> final_rs_path_, final_path_;
    double                       path_length_;

};   // Class definition

}   // namespace frontend
}   // namespace planning
