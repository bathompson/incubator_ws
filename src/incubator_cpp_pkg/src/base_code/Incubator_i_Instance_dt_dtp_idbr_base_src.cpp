#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_dt_dtp_idbr_base_src.hpp"

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

Incubator_i_Instance_dt_dtp_idbr_base::Incubator_i_Instance_dt_dtp_idbr_base() : Node("Incubator_i_Instance_dt_dtp_idbr")
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscription_options_.callback_group = cb_group_;

    // Setting up connections
    Incubator_i_Instance_dt_dtp_idbr_controller_status_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::ControllerStatusi>(
        "Incubator_i_Instance_dt_dtp_idbr_controller_status",
        1,
        std::bind(&Incubator_i_Instance_dt_dtp_idbr_base::handle_controller_status, this, std::placeholders::_1), subscription_options_);

    Incubator_i_Instance_dt_dtp_idbr_device_state_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::DeviceStatei>(
        "Incubator_i_Instance_dt_dtp_idbr_device_state",
        1,
        std::bind(&Incubator_i_Instance_dt_dtp_idbr_base::handle_device_state, this, std::placeholders::_1), subscription_options_);

}
