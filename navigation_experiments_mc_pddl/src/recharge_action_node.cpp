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

using namespace std::chrono_literals;

class RechargeAction : public plansys2::ActionExecutorClient
{
public:
  RechargeAction()
  : plansys2::ActionExecutorClient("charge", 500ms)
  {
    progress_ = 0.0;
    client_ = create_client<std_srvs::srv::Empty>("battery_contingency/battery_charged");
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) {
      RCLCPP_INFO(get_logger(), "Charing battery %f complete", progress_*100);
      progress_ += 0.05;
      send_feedback(progress_, "Charge running");
    } else {
      finish(true, 1.0, "Charge completed");
      srvCall();
      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Charging ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  void srvCall()
  {
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  auto result = client_->async_send_request(request);
}

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "recharge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
