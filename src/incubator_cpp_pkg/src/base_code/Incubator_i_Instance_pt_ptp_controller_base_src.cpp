#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_pt_ptp_controller_base_src.hpp"

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

Incubator_i_Instance_pt_ptp_controller_base::Incubator_i_Instance_pt_ptp_controller_base() : Node("Incubator_i_Instance_pt_ptp_controller")
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscription_options_.callback_group = cb_group_;

    // Setting up connections
    Incubator_i_Instance_pt_ptp_controller_device_state_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::DeviceStatei>(
        "Incubator_i_Instance_pt_ptp_controller_device_state",
        1,
        std::bind(&Incubator_i_Instance_pt_ptp_controller_base::handle_device_state, this, std::placeholders::_1), subscription_options_);

    Incubator_i_Instance_pt_ptp_controller_param_updates_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi>(
        "Incubator_i_Instance_pt_ptp_controller_param_updates",
        1,
        std::bind(&Incubator_i_Instance_pt_ptp_controller_base::handle_param_updates, this, std::placeholders::_1), subscription_options_);

    Incubator_i_Instance_pt_ptp_controller_request_heater_on_publisher_ = this->create_publisher<incubator_cpp_pkg_interfaces::msg::Boolean>(
        "Incubator_i_Instance_pt_ptp_device_manager_request_heater_on",
        1);

    Incubator_i_Instance_pt_ptp_controller_request_fan_on_publisher_ = this->create_publisher<incubator_cpp_pkg_interfaces::msg::Boolean>(
        "Incubator_i_Instance_pt_ptp_device_manager_request_fan_on",
        1);

    Incubator_i_Instance_pt_ptp_controller_controller_status_publisher_ = this->create_publisher<incubator_cpp_pkg_interfaces::msg::ControllerStatusi>(
        "Incubator_i_Instance_dt_dtp_idbr_controller_status",
        1);

}

//=================================================
//  C o m m u n i c a t i o n
//=================================================

void Incubator_i_Instance_pt_ptp_controller_base::put_request_heater_on(incubator_cpp_pkg_interfaces::msg::Boolean msg)
{
    Incubator_i_Instance_pt_ptp_controller_request_heater_on_publisher_->publish(msg);
}

void Incubator_i_Instance_pt_ptp_controller_base::put_request_fan_on(incubator_cpp_pkg_interfaces::msg::Boolean msg)
{
    Incubator_i_Instance_pt_ptp_controller_request_fan_on_publisher_->publish(msg);
}

void Incubator_i_Instance_pt_ptp_controller_base::put_controller_status(incubator_cpp_pkg_interfaces::msg::ControllerStatusi msg)
{
    Incubator_i_Instance_pt_ptp_controller_controller_status_publisher_->publish(msg);
}

