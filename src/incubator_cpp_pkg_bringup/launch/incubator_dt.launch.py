from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    db_recorder_node = Node(
        package="incubator_cpp_pkg",
        executable="Incubator_i_Instance_dt_dtp_idbr_exe"
    )

    lid_open_manager_node = Node(
        package="incubator_cpp_pkg",
        executable="Incubator_i_Instance_dt_dtp_loam_exe",
    )

    kalman_filter_node = Node(
        package="incubator_py_pkg",
        executable="Incubator_i_Instance_dt_dtp_pkf_exe"
    )

    return LaunchDescription([
        db_recorder_node,
        lid_open_manager_node,
        kalman_filter_node
    ])