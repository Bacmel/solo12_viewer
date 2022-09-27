from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command

from ament_index_python.packages import get_package_share_directory

import os.path
import numpy as np

default_urdf_path = os.path.join(get_package_share_directory('example-robot-data'), 'robots', 'solo_description', 'robots', 'solo12.urdf')
default_solo12_viewer_yaml_path = os.path.join(get_package_share_directory('solo12_viewer'), 'config', 'solo12_viewer.yaml')

default_debug_mode = 'false'

def generate_launch_description():
    
    urdf_path_arg = DeclareLaunchArgument('urdf_path', default_value=str(default_urdf_path), description="Path of the urdf file.")
    urdf_path = LaunchConfiguration('urdf_path')

    solo12_viewer_yaml_path_arg = DeclareLaunchArgument('solo12_viewer_yaml_path', default_value=str(default_solo12_viewer_yaml_path), description="Path of the yaml file with solo12_viewer node parameters.")
    solo12_viewer_yaml_path = LaunchConfiguration('solo12_viewer_yaml_path')

    debug_mode_arg = DeclareLaunchArgument('debug_mode', default_value=str(default_debug_mode), description="Boolean value the set the debug mode.")
    debug_mode = LaunchConfiguration('debug_mode')

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      output='log',
                                      parameters=[{'robot_description': ParameterValue(Command(['cat ',urdf_path]), value_type=str)}],
                                      arguments=[urdf_path])

    rviz_node                  = Node(package='rviz2',
                                      executable='rviz2',
                                      output='log',
                                      arguments=['-d' + os.path.join(get_package_share_directory('solo12_viewer'), 'config', 'config.rviz')],
                                      condition=UnlessCondition(debug_mode))
    
    custom_node                = Node(package='solo12_viewer',
                                      executable='solo12_viewer',
                                      output='both',
                                      parameters=[solo12_viewer_yaml_path])

    vrpn_client_node           = Node(package='vrpn_client_ros',
                                      executable='vrpn_client_node',
                                      output='both',
                                      emulate_tty=True,
                                      parameters=[{'server': '192.168.1.2', 'port': 3883, 'frame_id': 'world_fur', 'broadcast_tf': True, 'refresh_tracker_frequency': 0.2, 'update_frequency': 100.0}])

    st_w_wo                    = Node(package='tf2_ros',
                                      executable='static_transform_publisher',
                                      arguments=['0', '0', '0', '0', '0', str(np.pi/2), 'world_flu', 'world_fur'])

    st_solo12_fur_flu          = Node(package='tf2_ros',
                                      executable='static_transform_publisher',
                                      arguments=['0', '0', '0', '0', '0', str(-np.pi/2), 'solo12_fur_ot', 'solo12_flu_ot'])

    st_solo12_universe         = Node(package='tf2_ros',
                                      executable='static_transform_publisher',
                                      arguments=['0', '0', '0', '0', '0', '0', 'solo12_flu_ot', 'universe'])



    ld = LaunchDescription()

    ld.add_action(urdf_path_arg)
    ld.add_action(solo12_viewer_yaml_path_arg)
    ld.add_action(debug_mode_arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(custom_node)
    ld.add_action(vrpn_client_node)

    ld.add_action(st_w_wo)
    ld.add_action(st_solo12_fur_flu)
    ld.add_action(st_solo12_universe)

    return ld
