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

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(INIT)
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp_aux", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"f_normal_mode", "mode"});
    problem_expert_->addInstance(plansys2::Instance{"f_energy_saving_mode", "mode"});
    problem_expert_->addInstance(plansys2::Instance{"f_degraded_mode", "mode"});
    problem_expert_->addInstance(plansys2::Instance{"f_start_mode", "mode"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_enough r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(nav_sensor r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(charging_point_at wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_low_mode f_energy_saving_mode)"));
    problem_expert_->addPredicate(plansys2::Predicate("(normal_mode f_normal_mode)"));
    problem_expert_->addPredicate(plansys2::Predicate("(degraded_mode f_degraded_mode)"));
    problem_expert_->addPredicate(plansys2::Predicate("(current_system_mode f_start_mode)"));
  }

  void step()
  {
    switch (state_) {
      case INIT:
        {
          // Set the goal for next state, and execute plan
          problem_expert_->setGoal(plansys2::Goal("(and(current_system_mode f_normal_mode))"));

          if (executor_client_->executePlan()) {
            state_ = STARTING;
          }
          break;
        }
      case STARTING:
        {
          if (executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP1;
              }
            } else {
              executor_client_->executePlan();  // replan and execute
            }
          }
          break;    
        }
      case PATROL_WP1:
        {
          auto feedback = executor_client_->getFeedBack();
          
          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp1)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp2))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP2;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      case PATROL_WP2:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp2)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp3))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP3;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      case PATROL_WP3:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp3)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp4))"));

              if (executor_client_->executePlan()) {
                state_ = PATROL_WP4;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      case PATROL_WP4:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp4)"));

              // Set the goal for next state, and execute plan
              problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

              if (executor_client_->executePlan()) {
                // Loop to WP1
                state_ = PATROL_WP1;
              }
            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }
              executor_client_->executePlan();  // replan and execute
            }
          }
        }
        break;
      default:
        break;
    }
  }

private:
  typedef enum {INIT, STARTING, PATROL_WP1, PATROL_WP2, PATROL_WP3, PATROL_WP4} StateType;
  StateType state_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
