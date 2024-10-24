#include "rclcpp/rclcpp.hpp"
#include "incubator_cpp_pkg_interfaces/msg/boolean.hpp"
#include "incubator_cpp_pkg_interfaces/msg/device_statei.hpp"
#include "incubator_cpp_pkg_interfaces/msg/closed_loop_param_updatesi.hpp"
#include "incubator_cpp_pkg_interfaces/msg/controller_statusi.hpp"
#include <queue>

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

class Incubator_i_Instance_pt_ptp_controller_base : public rclcpp::Node
{
protected:
    Incubator_i_Instance_pt_ptp_controller_base();

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================

    #define PRINT_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
    #define PRINT_WARN(...) RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
    #define PRINT_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)

    void put_request_heater_on(incubator_cpp_pkg_interfaces::msg::Boolean msg);
    void put_request_fan_on(incubator_cpp_pkg_interfaces::msg::Boolean msg);
    void put_controller_status(incubator_cpp_pkg_interfaces::msg::ControllerStatusi msg);

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // SubscriptionOptions for assigning subscriptions to the callback group
    rclcpp::SubscriptionOptions subscription_options_;


    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    virtual void handle_device_state(const incubator_cpp_pkg_interfaces::msg::DeviceStatei::SharedPtr msg) = 0;
    virtual void handle_param_updates(const incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi::SharedPtr msg) = 0;

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::DeviceStatei>::SharedPtr Incubator_i_Instance_pt_ptp_controller_device_state_subscription_;
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi>::SharedPtr Incubator_i_Instance_pt_ptp_controller_param_updates_subscription_;

    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::Boolean>::SharedPtr Incubator_i_Instance_pt_ptp_controller_request_heater_on_publisher_;
    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::Boolean>::SharedPtr Incubator_i_Instance_pt_ptp_controller_request_fan_on_publisher_;
    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::ControllerStatusi>::SharedPtr Incubator_i_Instance_pt_ptp_controller_controller_status_publisher_;

};
