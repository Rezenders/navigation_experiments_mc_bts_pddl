import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    # Create the launch configuration variables
    # tomasys_file = LaunchConfiguration('tomasys_file')
    # desired_configuration = LaunchConfiguration('desired_configuration')

    tomasys_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'navigation_domain.owl')
    ]

    # tomasys_file_arg = DeclareLaunchArgument(
    #     'tomasys_file',
    #     default_value=str(tomasys_files_array),
    #     description='tomasys ontologies'
    # )

    # desired_configuration_arg = DeclareLaunchArgument(
    #     'desired_configuration',
    #     default_value='',
    #     description='Desired inital configuration (system mode)')

    model_file = os.path.join(pkg_mros_ontology_path, 'owl', 'urjc_pilot.owl')

    mros_reasoner_node = Node(
        package='mros2_reasoner',
        executable='mros2_reasoner_node',
        name='mros2_reasoner_node',
        output='screen',
        parameters=[{
            'tomasys_file': tomasys_files_array,
            'model_file': model_file,
            # 'desired_configuration': desired_configuration,
        }],
    )

    wrapper_node = Node(
        package='mros2_wrapper',
        executable='mros2_wrapper',
        name='mros2_wrapper',
        output='screen'
    )

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    return LaunchDescription([
        # tomasys_file_arg,
        # desired_configuration_arg,
        mros_reasoner_node,
        mros2_system_modes_bridge_node,
        wrapper_node
    ])
