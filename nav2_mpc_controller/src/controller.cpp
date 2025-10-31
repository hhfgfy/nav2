// Copyright (c) 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "nav2_mpc_controller/controller.hpp"

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace nav2_mpc_controller
{

MPCController::MPCController()
: logger_(rclcpp::get_logger("nav2_mpc_controller"))
{
}

void MPCController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Failed to lock node inside MPCController::configure");
  }

  name_ = name;
  logger_ = node->get_logger();
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  clock_ = node->get_clock();

  declareParameters(node);
  speed_limit_ = params_.max_linear_velocity;
  speed_limit_is_percentage_ = false;

  RCLCPP_INFO(logger_, "Configured MPC controller: %s", name_.c_str());
}

void MPCController::declareParameters(const rclcpp::Node::SharedPtr & node)
{
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prediction_horizon", rclcpp::ParameterValue(params_.prediction_horizon));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".time_step", rclcpp::ParameterValue(params_.time_step));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_linear_velocity", rclcpp::ParameterValue(params_.max_linear_velocity));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(params_.max_angular_velocity));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_lookahead_distance",
    rclcpp::ParameterValue(params_.path_lookahead_distance));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(params_.goal_tolerance));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_cost_weight", rclcpp::ParameterValue(params_.obstacle_cost_weight));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".orientation_cost_weight",
    rclcpp::ParameterValue(params_.orientation_cost_weight));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".velocity_cost_weight", rclcpp::ParameterValue(params_.velocity_cost_weight));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".linear_samples", rclcpp::ParameterValue(params_.linear_samples));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".angular_samples", rclcpp::ParameterValue(params_.angular_samples));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_reverse_motion",
    rclcpp::ParameterValue(params_.allow_reverse_motion));

  params_.prediction_horizon = static_cast<size_t>(std::max<int64_t>(
      1, node->get_parameter(name_ + ".prediction_horizon").as_int()));
  params_.time_step = std::max(0.01, node->get_parameter(name_ + ".time_step").as_double());
  params_.max_linear_velocity = std::max(0.01, node->get_parameter(
      name_ + ".max_linear_velocity").as_double());
  params_.max_angular_velocity = std::max(0.01, node->get_parameter(
      name_ + ".max_angular_velocity").as_double());
  params_.path_lookahead_distance = std::max(0.01, node->get_parameter(
      name_ + ".path_lookahead_distance").as_double());
  params_.goal_tolerance = std::max(0.0, node->get_parameter(name_ + ".goal_tolerance").as_double());
  params_.obstacle_cost_weight = std::max(0.0, node->get_parameter(
      name_ + ".obstacle_cost_weight").as_double());
  params_.orientation_cost_weight = std::max(0.0, node->get_parameter(
      name_ + ".orientation_cost_weight").as_double());
  params_.velocity_cost_weight = std::max(0.0, node->get_parameter(
      name_ + ".velocity_cost_weight").as_double());
  params_.linear_samples = static_cast<size_t>(std::max<int64_t>(
      1, node->get_parameter(name_ + ".linear_samples").as_int()));
  params_.angular_samples = static_cast<size_t>(std::max<int64_t>(
      1, node->get_parameter(name_ + ".angular_samples").as_int()));
  params_.allow_reverse_motion = node->get_parameter(
    name_ + ".allow_reverse_motion").as_bool();
}

void MPCController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up MPC controller: %s", name_.c_str());
  global_plan_.poses.clear();
}

void MPCController::activate()
{
  RCLCPP_INFO(logger_, "Activating MPC controller: %s", name_.c_str());
}

void MPCController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating MPC controller: %s", name_.c_str());
}

void MPCController::reset()
{
  RCLCPP_INFO(logger_, "Resetting MPC controller: %s", name_.c_str());
  global_plan_.poses.clear();
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::ControllerException("Received plan with zero poses in MPCController");
  }
  global_plan_ = path;
  RCLCPP_DEBUG(logger_, "Received plan with %zu poses", global_plan_.poses.size());
}

void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::ControllerException("Cannot compute command because global plan is empty");
  }

  const auto & goal_pose = global_plan_.poses.back().pose;
  if (goal_checker) {
    if (goal_checker->isGoalReached(robot_pose.pose, goal_pose, robot_speed)) {
      return buildStampedCommand(0.0, 0.0, robot_pose.header.stamp);
    }
  } else {
    const double goal_dist = nav2_util::geometry_utils::euclidean_distance(robot_pose.pose, goal_pose);
    if (goal_dist <= params_.goal_tolerance) {
      return buildStampedCommand(0.0, 0.0, robot_pose.header.stamp);
    }
  }

  const size_t closest_index = findClosestPoseIndex(robot_pose.pose);
  const size_t lookahead_index = computeLookaheadIndex(closest_index);

  auto result = optimizeControl(robot_pose, robot_speed, closest_index, lookahead_index);
  if (!std::isfinite(result.cost)) {
    throw nav2_core::ControllerException("MPC failed to find a valid control command");
  }

  return buildStampedCommand(
    result.commanded_linear_velocity,
    result.commanded_angular_velocity,
    robot_pose.header.stamp);
}

MPCTrajectory MPCController::optimizeControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  size_t closest_index, size_t lookahead_index) const
{
  MPCTrajectory best_trajectory;
  const double max_linear_velocity = [this]() {
      if (speed_limit_ <= 0.0) {
        return params_.max_linear_velocity;
      }
      if (speed_limit_is_percentage_) {
        return params_.max_linear_velocity * speed_limit_ * 0.01;
      }
      return std::min(params_.max_linear_velocity, speed_limit_);
    }();
  const double max_angular_velocity = params_.max_angular_velocity;

  const size_t linear_samples = std::max<size_t>(1, params_.linear_samples);
  const size_t angular_samples = std::max<size_t>(1, params_.angular_samples);
  const double linear_start = params_.allow_reverse_motion ? -max_linear_velocity : 0.0;
  const double linear_end = max_linear_velocity;
  const double angular_start = -max_angular_velocity;
  const double angular_end = max_angular_velocity;

  const double linear_step = linear_samples == 1 ? 0.0 : (linear_end - linear_start) /
    static_cast<double>(linear_samples - 1);
  const double angular_step = angular_samples == 1 ? 0.0 : (angular_end - angular_start) /
    static_cast<double>(angular_samples - 1);

  for (size_t i = 0; i < linear_samples; ++i) {
    const double linear_velocity = std::clamp(
      linear_start + linear_step * static_cast<double>(i),
      -max_linear_velocity, max_linear_velocity);

    for (size_t j = 0; j < angular_samples; ++j) {
      const double angular_velocity = std::clamp(
        angular_start + angular_step * static_cast<double>(j),
        -max_angular_velocity, max_angular_velocity);

      auto trajectory = rollOutTrajectory(robot_pose.pose, robot_speed, linear_velocity, angular_velocity);
      trajectory.cost = evaluateTrajectory(trajectory, closest_index, lookahead_index);

      if (trajectory.cost < best_trajectory.cost) {
        best_trajectory = trajectory;
        best_trajectory.commanded_linear_velocity = linear_velocity;
        best_trajectory.commanded_angular_velocity = angular_velocity;
      }
    }
  }

  return best_trajectory;
}

MPCTrajectory MPCController::rollOutTrajectory(
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Twist & start_speed,
  double linear_velocity, double angular_velocity) const
{
  MPCTrajectory result;
  result.commanded_linear_velocity = linear_velocity;
  result.commanded_angular_velocity = angular_velocity;

  double x = start_pose.position.x;
  double y = start_pose.position.y;
  double yaw = tf2::getYaw(start_pose.orientation);
  const double dt = params_.time_step;

  for (size_t k = 0; k < params_.prediction_horizon; ++k) {
    x += linear_velocity * std::cos(yaw) * dt;
    y += linear_velocity * std::sin(yaw) * dt;
    yaw = angles::normalize_angle(yaw + angular_velocity * dt);

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = start_pose.position.z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Twist vel = start_speed;
    vel.linear.x = linear_velocity;
    vel.angular.z = angular_velocity;

    result.states.push_back({pose, vel});
  }

  return result;
}

double MPCController::evaluateTrajectory(
  const MPCTrajectory & trajectory,
  size_t start_index, size_t target_index) const
{
  if (trajectory.states.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double cost = 0.0;
  const size_t plan_size = global_plan_.poses.size();
  const size_t clamped_target_index = std::min(target_index, plan_size - 1);

  for (size_t k = 0; k < trajectory.states.size(); ++k) {
    const double progress_ratio = static_cast<double>(k + 1) /
      static_cast<double>(trajectory.states.size());
    const size_t plan_index = std::min(
      clamped_target_index,
      start_index + static_cast<size_t>(std::round(progress_ratio * (clamped_target_index - start_index))));

    const auto & state = trajectory.states[k];
    const auto & reference_pose = global_plan_.poses[plan_index].pose;

    const double path_error = nav2_util::geometry_utils::euclidean_distance(state.pose, reference_pose);
    const double yaw_error = angles::shortest_angular_distance(
      tf2::getYaw(reference_pose.orientation), tf2::getYaw(state.pose.orientation));
    const double obstacle_cost = obstacleCostAtPose(state.pose);
    const double velocity_cost = std::abs(state.velocity.linear.x - params_.max_linear_velocity);

    cost += params_.obstacle_cost_weight * obstacle_cost;
    cost += params_.orientation_cost_weight * std::abs(yaw_error);
    cost += params_.velocity_cost_weight * velocity_cost;
    cost += path_error;
  }

  return cost;
}

size_t MPCController::findClosestPoseIndex(const geometry_msgs::msg::Pose & pose) const
{
  double best_distance = std::numeric_limits<double>::infinity();
  size_t best_index = 0;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double distance = nav2_util::geometry_utils::euclidean_distance(pose, global_plan_.poses[i].pose);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  return best_index;
}

size_t MPCController::computeLookaheadIndex(size_t start_index) const
{
  if (global_plan_.poses.empty()) {
    return 0;
  }

  double accumulated_distance = 0.0;
  size_t index = start_index;

  while (index + 1 < global_plan_.poses.size()) {
    const double segment = nav2_util::geometry_utils::euclidean_distance(
      global_plan_.poses[index].pose,
      global_plan_.poses[index + 1].pose);
    accumulated_distance += segment;
    ++index;

    if (accumulated_distance >= params_.path_lookahead_distance) {
      break;
    }
  }

  return std::min(index, global_plan_.poses.size() - 1);
}

double MPCController::obstacleCostAtPose(const geometry_msgs::msg::Pose & pose) const
{
  if (!costmap_) {
    return 0.0;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(pose.position.x, pose.position.y, mx, my)) {
    return nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  const unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

geometry_msgs::msg::TwistStamped MPCController::buildStampedCommand(
  double linear_velocity, double angular_velocity,
  const builtin_interfaces::msg::Time & stamp) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = stamp;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.twist.linear.x = linear_velocity;
  command.twist.linear.y = 0.0;
  command.twist.linear.z = 0.0;
  command.twist.angular.x = 0.0;
  command.twist.angular.y = 0.0;
  command.twist.angular.z = angular_velocity;
  return command;
}

}  // namespace nav2_mpc_controller

PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MPCController, nav2_core::Controller)
