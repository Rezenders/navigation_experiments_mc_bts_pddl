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
    pddl_dir = get_package_share_directory('navigation_experiments_mc_pddl')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('navigation_experiments_mc_pddl'),
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file': pddl_dir + '/pddl/patrol_domain_no_mc.pddl',
                'problem_file': pddl_dir + '/pddl/patrol_problem_no_mc.pddl',
            }.items()
        )

    # Specify the actions
    navigate_to_pose = Node(
        package='navigation_experiments_mc_pddl',
        executable='navigate_to_pose_action_node',
        name='navigate_to_pose_action_node',
        output='screen',
        parameters=[{
            'metacontrol': False,
            'action_name': 'navigate_to_pose'
        }]
    )

    navigate_to_recharge_station = Node(
        package='navigation_experiments_mc_pddl',
        executable='navigate_to_pose_action_node',
        name='navigate_to_recharge_station_action_node',
        output='screen',
        parameters=[{
            'metacontrol': False,
            'action_name': 'navigate_to_recharge_station'
        }]
    )

    navigate_to_recharge_station_degraded = Node(
        package='navigation_experiments_mc_pddl',
        executable='navigate_to_pose_action_node',
        name='navigate_to_recharge_station_degraded_action_node',
        output='screen',
        parameters=[{
            'metacontrol': False,
            'action_name': 'navigate_to_recharge_station_degraded'
        }]
    )

    navigate_to_pose_degraded = Node(
        package='navigation_experiments_mc_pddl',
        executable='navigate_to_pose_action_node',
        name='navigate_to_pose_degraded_action_node',
        output='screen',
        parameters=[{
            'metacontrol': False,
            'action_name': 'navigate_to_pose_degraded'
        }]
    )

    recharge_cmd = Node(
        package='navigation_experiments_mc_pddl',
        executable='recharge_action_node',
        name='recharge_action_node',
        output='screen',
        parameters=[])

    reconfigure_cmd = Node(
        package='navigation_experiments_mc_pddl',
        executable='reconfigure_action_node',
        name='reconfigure_action_node',
        output='screen',
        parameters=[{'action_name': 'reconfigure_system'}])

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

    ld.add_action(navigate_to_pose)
    ld.add_action(navigate_to_recharge_station)
    ld.add_action(navigate_to_recharge_station_degraded)
    ld.add_action(navigate_to_pose_degraded)
    ld.add_action(recharge_cmd)
    ld.add_action(reconfigure_cmd)
    # ld.add_action(pddl_controller_cmd)
    return ld
