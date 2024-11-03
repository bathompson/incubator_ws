#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_pt_ptp_device_manager_base_src.hpp"

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

Incubator_i_Instance_pt_ptp_device_manager_base::Incubator_i_Instance_pt_ptp_device_manager_base() : Node("Incubator_i_Instance_pt_ptp_device_manager")
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscription_options_.callback_group = cb_group_;

    // Setting up connections
    Incubator_i_Instance_pt_ptp_device_manager_request_heater_on_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::Boolean>(
        "Incubator_i_Instance_pt_ptp_device_manager_request_heater_on",
        1,
        std::bind(&Incubator_i_Instance_pt_ptp_device_manager_base::handle_request_heater_on, this, std::placeholders::_1), subscription_options_);

    Incubator_i_Instance_pt_ptp_device_manager_request_fan_on_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::Boolean>(
        "Incubator_i_Instance_pt_ptp_device_manager_request_fan_on",
        1,
        std::bind(&Incubator_i_Instance_pt_ptp_device_manager_base::handle_request_fan_on, this, std::placeholders::_1), subscription_options_);

    Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_1 = this->create_publisher<incubator_cpp_pkg_interfaces::msg::DeviceStatei>(
        "Incubator_i_Instance_dt_dtp_idbr_device_state",
        1);

    Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_2 = this->create_publisher<incubator_cpp_pkg_interfaces::msg::DeviceStatei>(
        "Incubator_i_Instance_dt_dtp_pkf_device_state",
        1);

    Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_3 = this->create_publisher<incubator_cpp_pkg_interfaces::msg::DeviceStatei>(
        "Incubator_i_Instance_pt_ptp_controller_device_state",
        1);

    // timeTriggered callback timer
    periodTimer_ = this->create_wall_timer(std::chrono::milliseconds(3000),
        std::bind(&Incubator_i_Instance_pt_ptp_device_manager_base::timeTriggered, this), cb_group_);

}

//=================================================
//  C o m m u n i c a t i o n
//=================================================

void Incubator_i_Instance_pt_ptp_device_manager_base::handle_request_heater_on(const incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr msg)
{
    request_heater_on_msg_holder = msg;
}

void Incubator_i_Instance_pt_ptp_device_manager_base::handle_request_fan_on(const incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr msg)
{
    request_fan_on_msg_holder = msg;
}

incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr Incubator_i_Instance_pt_ptp_device_manager_base::get_request_heater_on() {
    return request_heater_on_msg_holder;
}

incubator_cpp_pkg_interfaces::msg::Boolean::SharedPtr Incubator_i_Instance_pt_ptp_device_manager_base::get_request_fan_on() {
    return request_fan_on_msg_holder;
}

void Incubator_i_Instance_pt_ptp_device_manager_base::put_device_state(incubator_cpp_pkg_interfaces::msg::DeviceStatei msg)
{
    Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_1->publish(msg);
    Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_2->publish(msg);
    Incubator_i_Instance_pt_ptp_device_manager_device_state_publisher_3->publish(msg);
}

