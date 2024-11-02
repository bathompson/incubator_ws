from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    incubator_description_path = get_package_share_path('incubator_description')
    
    urdf_path = os.path.join(incubator_description_path, "urdf", "incubator.urdf.xacro")
    rviz_conf_path = os.path.join(incubator_description_path, "rviz", "urdf_config.rviz")

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_conf_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node
    ])