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

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

class PatrollingController : public rclcpp::Node
{
public:
  bool mission_completed = false;
  PatrollingController()
  : rclcpp::Node("patrolling_controller")
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10,
      std::bind(&PatrollingController::diagnostics_cb, this, _1));
  }

  void diagnostics_cb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    for(auto diagnostic_status : msg->status){
      if (diagnostic_status.message == "Component status"){
        auto component = diagnostic_status.values[0].key;
        auto value = diagnostic_status.values[0].value;
        if (component == "battery" && value == "FALSE"){
          problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_failure)"));
          problem_expert_->addPredicate(plansys2::Predicate("(battery_low r2d2)"));
          problem_expert_->removePredicate(plansys2::Predicate("(battery_charged r2d2)"));
        } else if(component == "laser_resender" && value == "FALSE"){
          problem_expert_->removePredicate(plansys2::Predicate("(nav_sensor r2d2)"));
        }
      }
    }
  }

  void step(){
    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
      if (executor_client_->getResult().value().success) {
        std::cout << "Successful finished " << std::endl;
        mission_completed = true;
      } else {
          std::cout << "Replanning!" << std::endl;
          execute_plan();
          return;
      }
    }

    auto feedback = executor_client_->getFeedBack();
    for (const auto & action_feedback : feedback.action_execution_status) {
      if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
        std::cout << "[" << action_feedback.action << "] finished with error: " <<
          action_feedback.message_status <<std::endl;
          break;
      }

      std::string arguments_str = " ";
      for (const auto & arguments: action_feedback.arguments){
        arguments_str += arguments + " ";
      }
      std::cout << "[" << action_feedback.action << arguments_str <<
        action_feedback.completion * 100.0 << "%]";
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

void execute_plan(){
  // Compute the plan
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    for (auto instance: problem_expert_->getInstances()){
      std::cout<<"Instance "<< instance.name.c_str() << " type " <<
        instance.type.c_str() << std::endl;
    }
    for (auto predicate: problem_expert_->getPredicates()) {
      std::cout << "Predicates: " << std::endl;
      std::cout << parser::pddl::toString(predicate)<<std::endl;
    }

    std::cout << "Could not find plan to reach goal " <<
     parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    return;
  }

  std::cout << "Selected plan: " << std::endl;
  for (auto item : plan->items){
    RCLCPP_INFO(this->get_logger(), "  Action: '%s'", item.action.c_str());
  }
  // Execute the plan
  executor_client_->start_plan_execution(plan.value());
}

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  rclcpp::Rate rate(5);
  node->execute_plan();
  while (rclcpp::ok() && !node->mission_completed) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
