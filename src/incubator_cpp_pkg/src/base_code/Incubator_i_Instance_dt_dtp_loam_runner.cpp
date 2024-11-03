#include "incubator_cpp_pkg/user_headers/Incubator_i_Instance_dt_dtp_loam_src.hpp"

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

Incubator_i_Instance_dt_dtp_loam::Incubator_i_Instance_dt_dtp_loam() : Incubator_i_Instance_dt_dtp_loam_base()
{
    // Invoke initialize entry point
    initialize();

    PRINT_INFO("Incubator_i_Instance_dt_dtp_loam infrastructure set up");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    auto node = std::make_shared<Incubator_i_Instance_dt_dtp_loam>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
