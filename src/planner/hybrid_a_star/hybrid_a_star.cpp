#include "hybrid_a_star.h"
#include "logger.h"
#include <cmath>


namespace planning {
namespace frontend {
HybridAstar::HybridAstar(const params::HybridAStarParams& hybrid_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collison_checker) {
    // 1. set the hybrid search params, map_ptr and collision_checker
    setParams(hybrid_params);
    setMap(map_ptr);
    setCollisionChecker(collison_checker);

    // 2. set the RS_path_ptr
    double turning_radius = hybrid_params_.wheel_base() / std::tan(hybrid_params_.max_steer_angle());
    setRSPath(std::make_unique<RSPath>(turning_radius));

    // 3. set hybrid astar node status
    const double map_min_x = map_ptr_->GetCostMap().map_info().min_x();
    const double map_min_y = map_ptr_->GetCostMap().map_info().min_y();
    const double map_max_x = map_ptr_->GetCostMap().map_info().max_x();
    const double map_max_y = map_ptr_->GetCostMap().map_info().max_y();

    const std::uint32_t HYBRID_ASTAR_X_SIZE =
        static_cast<std::uint32_t>(std::floor((map_max_x - map_min_x) / hybrid_params_.node_resolution_x()));
    const std::uint32_t HYBRID_ASTAR_Y_SIZE =
        static_cast<std::uint32_t>(std::floor((map_max_y - map_min_y) / hybrid_params_.node_resolution_y()));
    double resolution_theta =
        common::math::Angle<std::int32_t>::from_deg(hybrid_params_.node_resolution_theta()).to_rad();
    const std::uint32_t HYBRID_ASTAR_THETA_SIZE =
        static_cast<std::uint32_t>(std::floor((M_PI + M_PI) / resolution_theta));
    node_map_.resize(HYBRID_ASTAR_X_SIZE);
    for (uint32_t i = 0; i < HYBRID_ASTAR_X_SIZE; ++i) {
        node_map_[i].resize(HYBRID_ASTAR_Y_SIZE);
        for (uint32_t j = 0; j < HYBRID_ASTAR_Y_SIZE; ++j) {
            node_map_[i][j].resize(HYBRID_ASTAR_THETA_SIZE);
            for (uint32_t k = 0; k < HYBRID_ASTAR_THETA_SIZE; ++k) { node_map_[i][j][k] = nullptr; }
        }
    }
}

bool HybridAstar::Plan(const Eigen::Vector3d& start_vec, const Eigen::Vector3d& goal_vec) {
    start_vec_ = start_vec;
    goal_vec_  = goal_vec;
    LOG(INFO) << "Hybrid A* planner is running..."
              << "Start point: "
              << "x" << start_vec_.x() << "y" << start_vec_.y() << "theta" << start_vec_.z() << "Goal point: "
              << "x" << goal_vec_.x() << "y" << goal_vec_.y() << "theta" << goal_vec_.z();
    // 1. initial the first node
    initNode();

    bool reach_goal = false;
    int  iter       = 0;

    // 2. start plan
    while (!reach_goal && !open_list_.empty() && iter <= hybrid_params_.max_iter()) {
        // 2.1 select the node with the lowest f value
        selected_node_ = open_list_.top();
        open_list_.pop();
        debug_node_list_.push_back(Eigen::Vector3d(selected_node_->x_, selected_node_->y_, selected_node_->theta_));
        LOG(INFO) << "Hybrid A* planner is running..."
                  << "Current node: "
                  << "x is [" << selected_node_->x_ << "] y is [" << selected_node_->y_ << "] theta is ["
                  << selected_node_->theta_ << "]";
        // 2.2 check if the node can reach the goal with RS path
        if (tryRSPath(selected_node_)) {
            reach_goal = true;
            break;
        } else {
            // 2.3 expand the node
            expandNode(selected_node_);
        }
        ++iter;
    }

    // 3. post process and get the final path
    if (reach_goal) {
        LOG(INFO) << "Hybrid A* planner is finished..."
                  << "The path is found!";
        finishPath();
    } else {
        LOG(WARNING) << "Hybrid A* planner is finished..."
                     << "The path is not found!";
    }

    return reach_goal;
};

bool HybridAstar::tryRSPath(const std::shared_ptr<Node>& node_ptr) {
    // 0. check the node is in the rs radius
    double distance = std::sqrt((node_ptr->x_ - goal_vec_.x()) * (node_ptr->x_ - goal_vec_.x()) +
                                (node_ptr->y_ - goal_vec_.y()) * (node_ptr->y_ - goal_vec_.y()));
    if (distance > hybrid_params_.rs_radius()) { return false; }
    // 1. get the RS path
    double rs_path_length;
    auto   rs_path = rs_path_ptr_->GetRSPath(node_ptr->pose_, goal_vec_, hybrid_params_.rs_step_size(), rs_path_length);
    // 2. check if the RS path is valid (in the map && collision free)
    bool is_valid = true;
    for (const auto& rs_pose : rs_path) {
        if (isCollide(rs_pose.x(), rs_pose.y(), rs_pose.z())) {
            is_valid = false;
            break;
        }
    }
    // 3. if valid, store the rs_path
    if (is_valid) {
        final_rs_path_ = rs_path;
        // NOTE: the path_length_ is firstly set to the rs_path_length, and then add the hybrid path length
        path_length_ = rs_path_length;
        return true;
    }

    return false;
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
    selected_node_->setValue(0.0, 0.0);
    selected_node_->status_ = NODE_STATUS::OPEN;
    open_list_.push(selected_node_);
    std::uint32_t index_x, index_y, index_theta;
    convertToIndex(start_vec_.x(), start_vec_.y(), start_vec_.z(), index_x, index_y, index_theta);
    node_map_[index_x][index_y][index_theta] = selected_node_;
    LOG(INFO) << "inital node is created in node_map"
              << ", x is [" << node_map_[index_x][index_y][index_theta]->x_ << "]"
              << ", y is [" << node_map_[index_x][index_y][index_theta]->y_ << "]"
              << ", theta is [" << node_map_[index_x][index_y][index_theta]->theta_ << "]";
};

void HybridAstar::expandNode(const std::shared_ptr<Node>& node_ptr) {
    // 1. get the steer_angle range
    const double& max_steer_angle  = hybrid_params_.max_steer_angle();
    const int&    front_expand_num = hybrid_params_.search_front_num();
    const int&    back_expand_num  = hybrid_params_.search_back_num();
    const int     expand_num       = front_expand_num + back_expand_num;
    // 2. expand the node
    for (int i = 0; i < expand_num; ++i) {
        double expand_s, is_forward, child_steer_angle;
        if (i < front_expand_num) {
            expand_s          = hybrid_params_.expand_s();
            is_forward        = true;
            child_steer_angle = -max_steer_angle + i * 2 * max_steer_angle / (front_expand_num - 1);
        } else {
            expand_s          = -hybrid_params_.expand_s();
            is_forward        = false;
            child_steer_angle = -max_steer_angle + (i - front_expand_num) * 2 * max_steer_angle / (back_expand_num - 1);
        }
        double                       child_x, child_y, child_theta;
        std::vector<Eigen::Vector3d> path_from_parent;
        if (!getChildNode(expand_s, child_steer_angle, child_x, child_y, child_theta, node_ptr, path_from_parent)) {
            continue;
        }
        std::uint32_t index_x, index_y, index_theta;
        convertToIndex(child_x, child_y, child_theta, index_x, index_y, index_theta);
        if (node_map_[index_x][index_y][index_theta] == nullptr) {
            // 2.2 create the new node
            auto child_node_ptr = std::make_shared<Node>(
                child_x, child_y, child_theta, child_steer_angle, is_forward, node_ptr, path_from_parent);
            double child_g_value = calcGValue(node_ptr, child_node_ptr);
            double child_h_value = calcHValue(child_node_ptr);
            child_node_ptr->setValue(child_g_value, child_h_value);
            child_node_ptr->status_ = NODE_STATUS::OPEN;
            open_list_.push(child_node_ptr);
            node_map_[index_x][index_y][index_theta] = child_node_ptr;
        } else if (node_map_[index_x][index_y][index_theta]->status_ == NODE_STATUS::OPEN) {
            double new_child_g_value = calcGValue(node_ptr, node_map_[index_x][index_y][index_theta]);
            double new_child_h_value = calcHValue(node_map_[index_x][index_y][index_theta]);
            double new_f_value       = new_child_g_value + new_child_h_value;
            if (new_f_value < node_map_[index_x][index_y][index_theta]->f_value_) {
                // 2.2 update the node
                node_map_[index_x][index_y][index_theta]->setValue(new_child_g_value, new_child_h_value);
                node_map_[index_x][index_y][index_theta]->parent_ptr_  = node_ptr;
                node_map_[index_x][index_y][index_theta]->steer_angle_ = child_steer_angle;
                node_map_[index_x][index_y][index_theta]->is_forward_  = is_forward;
                node_map_[index_x][index_y][index_theta]->path_from_parent_.clear();
                node_map_[index_x][index_y][index_theta]->path_from_parent_ = path_from_parent;
            }
        } else if (node_map_[index_x][index_y][index_theta]->status_ == NODE_STATUS::CLOSE) {
            continue;
        }
    }

    // 3. close the node
    node_ptr->status_ = NODE_STATUS::CLOSE;
};

/***
 * @description: get the child node state (x, y, theta)
 * @param {double} expand_s
 * @param {double} steer_angle
 * @param {double&} child_x
 * @param {double&} child_y
 * @param {double&} child_theta
 * @param {std::shared_ptr<Node>&} node_ptr
 * @return {bool} true if success, otherwise false
 */
bool HybridAstar::getChildNode(const double expand_s, const double steer_angle, double& child_x, double& child_y,
                               double& child_theta, const std::shared_ptr<Node>& node_ptr,
                               std::vector<Eigen::Vector3d>& path_from_parent) {
    const double& wheel_base = hybrid_params_.wheel_base();
    for (std::uint32_t step_num = 0; step_num < hybrid_params_.expand_step_num(); ++step_num) {
        double expand_s_step = expand_s / hybrid_params_.expand_step_num() * (step_num + 1);
        child_theta =
            common::math::NormalizeAngle(node_ptr->theta_ + std::tan(steer_angle) * expand_s_step / wheel_base);
        child_x = node_ptr->x_ + expand_s_step * std::cos(child_theta);
        child_y = node_ptr->y_ + expand_s_step * std::sin(child_theta);
        path_from_parent.push_back(Eigen::Vector3d(child_x, child_y, child_theta));
        if (isCollide(child_x, child_y, child_theta)) { return false; }
    }
    path_from_parent.pop_back();
    return true;
};

bool HybridAstar::isCollide(double x, double y, double theta) {
    for (const auto& obstacle_polygon : map_ptr_->GetObsList()) {
        if (collision_checker_->Check(obstacle_polygon, common::math::Pose(x, y, theta))) { return true; }
    }
    if (!map_ptr_->IsInMap(x, y)) { return true; }

    return false;
};

double HybridAstar::calcHValue(const std::shared_ptr<Node>& child_node_ptr) {
    double h_value = 0.0;
    // use Manhattan distance as the heuristic function
    double to_goal_dis = std::sqrt((child_node_ptr->x_ - goal_vec_.x()) * (child_node_ptr->x_ - goal_vec_.x()) +
                                   (child_node_ptr->y_ - goal_vec_.y()) * (child_node_ptr->y_ - goal_vec_.y()));
    if (to_goal_dis < hybrid_params_.rs_radius()) {
        double rs_path_length = rs_path_ptr_->Distance(child_node_ptr->x_,
                                                       child_node_ptr->y_,
                                                       child_node_ptr->theta_,
                                                       goal_vec_.x(),
                                                       goal_vec_.y(),
                                                       goal_vec_.z());
        h_value += rs_path_length;
    } else {
        h_value += std::abs(child_node_ptr->x_ - goal_vec_.x()) + std::abs(child_node_ptr->y_ - goal_vec_.y());
    }

    return h_value;
};

double HybridAstar::calcGValue(const std::shared_ptr<Node>& curr_node_ptr,
                               const std::shared_ptr<Node>& child_node_ptr) {
    double g_value = 0.0;
    if (!child_node_ptr->is_forward_) { g_value += hybrid_params_.g_cost().reverse_cost(); }
    const double head_change = std::abs(common::math::NormalizeAngle(child_node_ptr->theta_ - curr_node_ptr->theta_));
    g_value += hybrid_params_.g_cost().head_change_cost() * head_change;
    return g_value;
};

void HybridAstar::finishPath() {
    // 1. get the revert path
    while (selected_node_ != nullptr) {
        final_path_.push_back(Eigen::Vector3d(selected_node_->x_, selected_node_->y_, selected_node_->theta_));
        for (auto i = selected_node_->path_from_parent_.rbegin(); i != selected_node_->path_from_parent_.rend(); ++i) {
            final_path_.push_back(*i);
        }
        if (selected_node_->parent_ptr_ != nullptr) {
            path_length_ += std::sqrt((selected_node_->x_ - selected_node_->parent_ptr_->x_) *
                                          (selected_node_->x_ - selected_node_->parent_ptr_->x_) +
                                      (selected_node_->y_ - selected_node_->parent_ptr_->y_) *
                                          (selected_node_->y_ - selected_node_->parent_ptr_->y_));
        }
        selected_node_ = selected_node_->parent_ptr_;
    }
    std::reverse(final_path_.begin(), final_path_.end());
    final_path_.insert(final_path_.end(), final_rs_path_.begin(), final_rs_path_.end());
};

}   // namespace frontend
}   // namespace planning