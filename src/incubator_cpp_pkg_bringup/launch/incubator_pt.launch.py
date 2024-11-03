from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    incubator_controller_node = Node(
        package="incubator_cpp_pkg",
        executable="Incubator_i_Instance_pt_ptp_controller_exe"
    )

    incubator_device_manager_node = Node(
        package="incubator_cpp_pkg",
        executable="Incubator_i_Instance_pt_ptp_device_manager_exe",
    )

    return LaunchDescription([
        incubator_controller_node,
        incubator_device_manager_node
    ])