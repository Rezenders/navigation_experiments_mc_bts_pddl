# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray
from system_modes_msgs.msg import ModeEvent


class ReconfigTime(Node):
    def __init__(self, last_contexts=None):
        super().__init__('reconfig_time')
        self.diagnostics_sub_ = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.diagnostics_cb, 1)

        self.mode_sub_ = self.create_subscription(
            ModeEvent,
            "/f_navigate/mode_request_info",
            self.mode_cb, 1)

        self.pub_ = self.create_publisher(
            Float64,
            '/navigation_experiments_mc_bts_pddl/reconfig_time',
            10)

        self.reconfig_time_ = 0.0
        self.component_in_error_time_ = 0.0
        self.last_mode_ = ""
        self.error_detected = False

    def destroy(self):
        super().destroy_node()

    def mode_cb(self, msg):
        if self.error_detected is True and (
         msg.goal_mode.label == 'f_energy_saving_mode' or
         msg.goal_mode.label == 'f_degraded_mode'):
            self.error_detected = False
            self.reconfig_time_ = self.get_clock().now() - \
                self.component_in_error_time_
            time_msg = Float64()
            time_msg.data = self.reconfig_time_.to_msg().sec + (
                self.reconfig_time_.to_msg().nanosec / 1000000000)
            self.pub_.publish(time_msg)
        # self.last_mode_ = msg.goal_mode.label

    def diagnostics_cb(self, msg):
        for diagnostic_status in msg.status:
            # 2 types of diagnostics considered: about bindings in error (TODO not implemented yet) or about QAs
            if diagnostic_status.message == "binding error":
                self.get_logger().warning("Diagnostics message received for %s with level %d, nothing done about it." % (diagnostic_status.name, diagnostic_status.level))

            # Component error
            elif diagnostic_status.message == "Component status":
                self.get_logger().info("\nCS Message received!\tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                component = diagnostic_status.values[0].key
                value = diagnostic_status.values[0].value
                self.get_logger().info("Component: {0} - Value {1}".format(component, value))
                if component == "battery":
                    if value == "FALSE":
                        self.component_in_error_time_ = self.get_clock().now()
                        self.error_detected = True
                elif component == "laser_resender":
                    if value == "FALSE":
                        self.component_in_error_time_ = self.get_clock().now()
                        self.error_detected = True


def main(args=None):
    rclpy.init(args=args)
    node = ReconfigTime()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
