#include "planner/hybrid_a_star/hybrid_a_star.h"
#include "common/math/math_utils.h"
#include <cmath>
#include <cstdint>
#include <memory>


namespace planning {
namespace frontend {
HybridAstar::HybridAstar(const params::HybridAStarParams& hybrid_params, const std::shared_ptr<map::Map>& map_ptr) {
    setParams(hybrid_params);
    setMap(map_ptr);
}

bool HybridAstar::Plan(const Eigen::Vector3d& start_vec, const Eigen::Vector3d& goal_vec) {
    start_vec_ = start_vec;
    goal_vec_  = goal_vec;
    LOG(INFO) << "Hybrid A* planner is running..."
              << "Start point: "
              << "x" << start_vec_.x() << "y" << start_vec_.y() << "theta" << start_vec_.z() << "Goal point: "
              << "x" << goal_vec_.x() << "y" << goal_vec_.y() << "theta" << goal_vec_.z();
    // 1. initial the node
    initNode();
    bool reach_goal = false;
    int  iter       = 0;

    // 2. start plan
    while (!reach_goal && !open_list_.empty() && iter <= hybrid_params_.max_iter()) {
        // 2.1 select the node with the lowest f value
        selected_node_ = open_list_.top();
        open_list_.pop();
        // 2.2 check if the node is the goal
        if (isReach(selected_node_)) {
            reach_goal = true;
            break;
        }
        // 2.3 expand the node
    }


    return reach_goal;
};

bool HybridAstar::isReach(const std::shared_ptr<Node>& node_ptr) {
    if (std::abs(node_ptr->x_ - goal_vec_.x()) < kEpsilon && std::abs(node_ptr->y_ - goal_vec_.y()) < kEpsilon &&
        std::abs(node_ptr->theta_ - goal_vec_.z()) < kEpsilon) {
        return true;
    } else {
        return false;
    }
};

void HybridAstar::initNode() {
    selected_node_ = std::make_shared<Node>(start_vec_.x(), start_vec_.y(), start_vec_.z(), 0.0, true, nullptr);
    selected_node_->updateValue(0.0, 0.0);
    open_list_.push(std::move(selected_node_));
};

void HybridAstar::ExpandNode(const std::shared_ptr<Node>& node_ptr) {
    // 1. get the steer_angle range
    const double& max_steer_angle  = hybrid_params_.max_steer_angle();
    const double& wheel_base       = hybrid_params_.wheel_base();
    const int&    front_expand_num = hybrid_params_.search_front_num();
    const int&    back_expand_num  = hybrid_params_.search_back_num();
    const int     expand_num       = front_expand_num + back_expand_num;
    // 2. expand the node
    for (int i = 0; i < expand_num; ++i) {
        const double expand_s          = i < front_expand_num ? hybrid_params_.expand_s() : -hybrid_params_.expand_s();
        bool         is_forward        = i < front_expand_num ? true : false;
        double       child_steer_angle = -max_steer_angle + i * max_steer_angle / front_expand_num;
        double       child_theta =
            common::math::NormalizeAngle(node_ptr->theta_ + std::tan(child_steer_angle) * expand_s / wheel_base);
        // 2.1 update the state
        double child_x = node_ptr->x_ + expand_s * std::cos(child_theta);
        double child_y = node_ptr->y_ + expand_s * std::sin(child_theta);
        auto   child_node_ptr =
            std::make_shared<Node>(child_x, child_y, child_theta, child_steer_angle, is_forward, node_ptr);
        // 2.2 check if the node is in the map or in the closed list, if not, check it
        if (!isClosedList(child_node_ptr)) {
            if (!isOpenList(child_node_ptr)) {
                // 2.3 check collision and update value of the node
            } else {
                // 2.4 update the value of the visited node
            }
        }
    }
};

}   // namespace frontend
}   // namespace planning