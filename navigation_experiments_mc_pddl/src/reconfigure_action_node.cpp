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

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "system_modes_msgs/srv/change_mode.hpp"
#include "system_modes_msgs/msg/mode_event.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class ReconfigureAction : public plansys2::ActionExecutorClient
{
public:
  ReconfigureAction()
  : plansys2::ActionExecutorClient("reconfig_system", 500ms)
  {
    mode_ = "";
    client_cb_group_ =  this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = create_client<system_modes_msgs::srv::ChangeMode>(
      "/f_navigate/change_mode", rmw_qos_profile_services_default, client_cb_group_);

    mode_sub_ = create_subscription<system_modes_msgs::msg::ModeEvent>(
      "/f_navigate/mode_request_info",
      10,
      std::bind(&ReconfigureAction::mode_cb, this, _1));
    current_mode_ = "deactivated";
  }

  void mode_cb(const system_modes_msgs::msg::ModeEvent::SharedPtr msg)
  {
    current_mode_ = msg->goal_mode.label;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  void do_work()
  {
    mode_ = get_arguments()[2];
    if (current_mode_ == mode_){
      RCLCPP_INFO(get_logger(), "Reconfiguration finished! Current mode %s", current_mode_.c_str());
      finish(true, 1.0, "System reconfigured!");
      return;
    }else{
      RCLCPP_INFO(get_logger(), "Reconfiguring system mode to %s", mode_.c_str());
      bool reconfiguration_result = srvCall();
      if (reconfiguration_result == false){
        finish(false, 1.0, "Reconfiguration failed");
        RCLCPP_INFO(get_logger(), "Reconfiguration failed! Current mode %s ", current_mode_.c_str());
      }
    }

  }

  bool srvCall()
  {
    auto request = std::make_shared<system_modes_msgs::srv::ChangeMode::Request>();
    request->mode_name = mode_;

    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    auto result_future = client_->async_send_request(request);
    std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
    if (status == std::future_status::ready) {
      return true;
    }
    return false;
  }

  rclcpp::Subscription<system_modes_msgs::msg::ModeEvent>::SharedPtr mode_sub_;
  std::string current_mode_;

  rclcpp::Client<system_modes_msgs::srv::ChangeMode>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  std::string mode_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReconfigureAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "reconfig_system"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
