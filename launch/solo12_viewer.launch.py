from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

default_urdf_path = '/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf'

def generate_launch_description():
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value=str(default_urdf_path), description="Path of the urdf file")

    urdf_path = LaunchConfiguration('urdf_path')

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      output='both',
                                      parameters=[{'robot_description': ParameterValue(Command(['cat ',urdf_path]), value_type=str)}],
                                      arguments=[urdf_path])

    rviz_node                  = Node(package='rviz2',
                                      executable='rviz2',
                                      output='both')
    
    custom_node                = Node(package='solo12_viewer',
                                      executable='solo12_viewer',
                                      output='both')


    ld = LaunchDescription()

    ld.add_action(urdf_path_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(custom_node)

    return ld
