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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('navigation_experiments_mc_pddl')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('navigation_experiments_mc_pddl'),
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file': example_dir + '/pddl/patrol_domain.pddl',
                'problem_file': example_dir + '/pddl/patrol_problem.pddl',
            }.items()
        )

    # Specify the actions
    move_cmd = Node(
        package='navigation_experiments_mc_pddl',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[])

    patrol_cmd = Node(
        package='navigation_experiments_mc_pddl',
        executable='patrol_action_node',
        name='patrol_action_node',
        output='screen',
        parameters=[])

    recharge_cmd = Node(
        package='navigation_experiments_mc_pddl',
        executable='recharge_action_node',
        name='recharge_action_node',
        output='screen',
        parameters=[])

    move_recharge_station_cmd = Node(
        package='navigation_experiments_mc_pddl',
        executable='move_recharge_station_action_node',
        name='move_recharge_station_action_node',
        output='screen',
        parameters=[])

    # pddl_controller_cmd = Node(
    #    package='navigation_experiments_mc_pddl',
    #    executable='patrolling_controller_node',
    #    name='patrolling_controller_node',
    #    output='screen',
    #    parameters=[])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(patrol_cmd)
    ld.add_action(recharge_cmd)
    ld.add_action(move_recharge_station_cmd)
    # ld.add_action(pddl_controller_cmd)
    return ld
