#include "rclcpp/rclcpp.hpp"
#include "incubator_cpp_pkg_interfaces/msg/controller_statusi.hpp"
#include "incubator_cpp_pkg_interfaces/msg/device_statei.hpp"
#include <queue>

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

class Incubator_i_Instance_dt_dtp_idbr_base : public rclcpp::Node
{
protected:
    Incubator_i_Instance_dt_dtp_idbr_base();

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================

    #define PRINT_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
    #define PRINT_WARN(...) RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
    #define PRINT_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)


private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // SubscriptionOptions for assigning subscriptions to the callback group
    rclcpp::SubscriptionOptions subscription_options_;


    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    virtual void handle_controller_status(const incubator_cpp_pkg_interfaces::msg::ControllerStatusi::SharedPtr msg) = 0;
    virtual void handle_device_state(const incubator_cpp_pkg_interfaces::msg::DeviceStatei::SharedPtr msg) = 0;

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::ControllerStatusi>::SharedPtr Incubator_i_Instance_dt_dtp_idbr_controller_status_subscription_;
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::DeviceStatei>::SharedPtr Incubator_i_Instance_dt_dtp_idbr_device_state_subscription_;


};
