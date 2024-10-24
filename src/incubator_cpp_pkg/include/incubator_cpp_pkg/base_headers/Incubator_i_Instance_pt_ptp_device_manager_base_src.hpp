#include "rclcpp/rclcpp.hpp"
#include "incubator_cpp_pkg_interfaces/msg/boolean.hpp"
#include "incubator_cpp_pkg_interfaces/msg/device_statei.hpp"
#include <queue>

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

class Incubator_i_Instance_pt_ptp_device_manager_base : public rclcpp::Node
{
protected:
    Incubator_i_Instance_pt_ptp_device_manager_base();

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================

    #define PRINT_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
    #define PRINT_WARN(...) RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
    #define PRINT_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)

    void put_device_state(incubator_cpp_pkg_interfaces::msg::DeviceStatei msg);

    incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr get_request_heater_on();
    incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr get_request_fan_on();

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // SubscriptionOptions for assigning subscriptions to the callback group
    rclcpp::SubscriptionOptions subscription_options_;


    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    void handle_request_heater_on(const incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr msg);
    void handle_request_fan_on(const incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr msg);

    incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr request_heater_on_msg_holder;
    incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr request_fan_on_msg_holder;

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::Boolean>::SharedPtr Incubator_i_Instance_pt_ptp_device_manager_request_heater_on_subscription_;
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::Boolean>::SharedPtr Incubator_i_Instance_pt_ptp_device_manager_request_fan_on_subscription_;

    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::DeviceStatei>::SharedPtr Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_1;
    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::DeviceStatei>::SharedPtr Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_2;

    //=================================================
    //  C a l l b a c k   a n d   T i m e r
    //=================================================
    virtual void timeTriggered() = 0;

    rclcpp::TimerBase::SharedPtr periodTimer_;

};
