// Copyright 2019 Intelligent Robotics Lab
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

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mros2_msgs/action/navigate_to_pose_qos.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using NavigateToPoseQos = mros2_msgs::action::NavigateToPoseQos;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "/map";
    wp.pose.position.x = 1.0;
    wp.pose.position.y = -1.0;
    wp.pose.position.z = 0.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);

    waypoints_["wp1"] = wp;

    wp.pose.position.x = -1.0;
    wp.pose.position.y = 1.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI);
    waypoints_["wp2"] = wp;

    wp.pose.position.x = -3.5;
    wp.pose.position.y = 1.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI);
    waypoints_["wp3"] = wp;

    wp.pose.position.x = -6.25;
    wp.pose.position.y = 2.66;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-M_PI_2);
    waypoints_["wp4"] = wp;

    wp.pose.position.x = -6.40;
    wp.pose.position.y = -2.81;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-M_PI_2);
    waypoints_["wp5"] = wp;

    wp.pose.position.x = -6.25;
    wp.pose.position.y = 2.66;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);
    waypoints_["wp6"] = wp;

    wp.pose.position.x = -1.5;
    wp.pose.position.y = 1.5;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
    waypoints_["wp7"] = wp;

    wp.pose.position.x = 4.0;
    wp.pose.position.y = -3.0;
    wp.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
    waypoints_["wp_r"] = wp;


    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));

    waypoint_follower_state_ = "inactive";
    client_cb_group_ =  this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    waypoint_follower_state_client_ = create_client<lifecycle_msgs::srv::GetState>(
      "/waypoint_follower/get_state", rmw_qos_profile_services_default, client_cb_group_);
    // waypoint_follower_state_sub = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    //   "/waypoint_follower/transition_event",
    //   10,
    //   std::bind(&MoveAction::waypoint_cb, this, _1));

    this->declare_parameter("metacontrol", true);
    // problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    use_metacontrol = false;
    nav2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  std::string get_waypoint_follower_state(){
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    while (!waypoint_follower_state_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return "";
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    auto result_future = waypoint_follower_state_client_->async_send_request(request);
    std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
    if (status == std::future_status::ready) {
      return result_future.get()->current_state.label;
    }
    return "";
  }

  void send_mc_navigation_goal(){
    navigation_action_client_ =
      rclcpp_action::create_client<NavigateToPoseQos>(
      shared_from_this(),
      "navigate_to_pose_qos");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    wp_to_navigate_ = get_arguments()[2];  // The goal is in the 3rd argument of the action

    RCLCPP_INFO(get_logger(), "Request navigation to [%s]", wp_to_navigate_.c_str());

    goal_pos_ = waypoints_[wp_to_navigate_];
    navigation_goal_.pose = goal_pos_;
    navigation_goal_.qos_expected.objective_type = "f_navigate"; // should be mros_goal->qos_expected.objective_type = "f_navigate";
    diagnostic_msgs::msg::KeyValue energy_qos;
    energy_qos.key = "energy";
    energy_qos.value = "0.7";
    diagnostic_msgs::msg::KeyValue safety_qos;
    safety_qos.key = "safety";
    safety_qos.value = "0.5";
    navigation_goal_.qos_expected.qos.clear();
    navigation_goal_.qos_expected.qos.push_back(energy_qos);
    navigation_goal_.qos_expected.qos.push_back(safety_qos);
    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPoseQos>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
    NavigationGoalHandle::SharedPtr,
    NavigationFeedback feedback) {
      send_feedback(
        std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
        "Move running");
    };

    send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "Move completed");
    };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate_.c_str());
  }

  void send_nav2_goal(){
    waypoint_follower_state_ = get_waypoint_follower_state();
    while (waypoint_follower_state_ != "active") {
      waypoint_follower_state_ = get_waypoint_follower_state();
      RCLCPP_INFO(get_logger(), "Waiting for waypoint_follower... %s", waypoint_follower_state_.c_str());
      rclcpp::sleep_for(1s);
    }

    nav2_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        nav2_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Request navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    nav2_goal_.pose = goal_pos_;
    nav2_goal_.pose = goal_pos_;
    nav2_goal_.pose.header.stamp = now();

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      Nav2GoalHandle::SharedPtr,
      Nav2Feedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "Move completed");
    };

    future_nav2_goal_handle_ =
      nav2_client_->async_send_goal(nav2_goal_, send_goal_options);
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    use_metacontrol = this->get_parameter("metacontrol").as_bool();
    send_feedback(0.0, "Move starting");
    if(use_metacontrol == true){
      send_mc_navigation_goal();
    } else{
      send_nav2_goal();
    }

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    if(use_metacontrol){
      navigation_action_client_->async_cancel_all_goals();
      return ActionExecutorClient::on_deactivate(previous_state);
    }
    nav2_client_->async_cancel_all_goals();

   return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work() { }

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPoseQos>;
  using NavigationFeedback =
    const std::shared_ptr<const NavigateToPoseQos::Feedback>;

  using Nav2GoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using Nav2Feedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<NavigateToPoseQos>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  NavigateToPoseQos::Goal navigation_goal_;

  double dist_to_move;
  std::string wp_to_navigate_;

  bool use_metacontrol;
  // NAV2
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  std::shared_future<Nav2GoalHandle::SharedPtr> future_nav2_goal_handle_;
  rclcpp::CallbackGroup::SharedPtr nav2_cb_group_;
  nav2_msgs::action::NavigateToPose::Goal nav2_goal_;

  std::string waypoint_follower_state_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr waypoint_follower_state_client_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
