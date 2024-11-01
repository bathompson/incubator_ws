#include "incubator_cpp_pkg/user_headers/Incubator_i_Instance_pt_ptp_device_manager_src.hpp"
#include <pigpio.h>
#include <signal.h>
//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

Incubator_i_Instance_pt_ptp_device_manager::Incubator_i_Instance_pt_ptp_device_manager() : Incubator_i_Instance_pt_ptp_device_manager_base()
{
    // Invoke initialize entry point
    initialize();

    PRINT_INFO("Incubator_i_Instance_pt_ptp_device_manager infrastructure set up");
}

std::shared_ptr<Incubator_i_Instance_pt_ptp_device_manager> mainNode;

void sigintHandler(int c)
{
    mainNode->takeDownDevices();
    gpioTerminate();
    exit(0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    gpioInitialise();
    auto node = std::make_shared<Incubator_i_Instance_pt_ptp_device_manager>();
    mainNode = node;
    signal(SIGINT, sigintHandler);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
