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

#ifndef NAV2_MPC_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPC_CONTROLLER__CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_mpc_controller
{

/**
 * @struct MPCParameters
 * @brief Tunable parameters that define the behaviour of the MPC loop.
 */
struct MPCParameters
{
  size_t prediction_horizon{10};
  double time_step{0.1};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.0};
  double path_lookahead_distance{1.0};
  double goal_tolerance{0.25};
  double obstacle_cost_weight{1.0};
  double orientation_cost_weight{1.0};
  double velocity_cost_weight{0.1};
  size_t linear_samples{5};
  size_t angular_samples{5};
  bool allow_reverse_motion{false};
};

/**
 * @struct MPCTrajectoryState
 * @brief Single state contained in a predicted trajectory rollout.
 */
struct MPCTrajectoryState
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist velocity;
};

/**
 * @struct MPCTrajectory
 * @brief Result of an MPC rollout along with its accumulated cost.
 */
struct MPCTrajectory
{
  std::vector<MPCTrajectoryState> states;
  double cost{std::numeric_limits<double>::infinity()};
  double commanded_linear_velocity{0.0};
  double commanded_angular_velocity{0.0};
};

/**
 * @class MPCController
 * @brief Simple MPC-based controller implementing the nav2_core controller interface.
 */
class MPCController : public nav2_core::Controller
{
public:
  MPCController();

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  void reset() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using Costmap = nav2_costmap_2d::Costmap2D;

  MPCTrajectory optimizeControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    size_t closest_index, size_t lookahead_index) const;

  MPCTrajectory rollOutTrajectory(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Twist & start_speed,
    double linear_velocity, double angular_velocity) const;

  double evaluateTrajectory(
    const MPCTrajectory & trajectory,
    size_t start_index, size_t target_index) const;

  size_t findClosestPoseIndex(const geometry_msgs::msg::Pose & pose) const;

  size_t computeLookaheadIndex(size_t start_index) const;

  double obstacleCostAtPose(const geometry_msgs::msg::Pose & pose) const;

  void declareParameters(const rclcpp::Node::SharedPtr & node);

  geometry_msgs::msg::TwistStamped buildStampedCommand(
    double linear_velocity, double angular_velocity,
    const builtin_interfaces::msg::Time & stamp) const;

  nav_msgs::msg::Path global_plan_;
  MPCParameters params_;
  std::string name_;
  nav2::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  Costmap * costmap_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Clock::SharedPtr clock_;
  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};
};

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__CONTROLLER_HPP_
