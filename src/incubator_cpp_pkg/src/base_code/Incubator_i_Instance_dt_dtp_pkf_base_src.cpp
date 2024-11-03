#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_dt_dtp_pkf_base_src.hpp"

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

Incubator_i_Instance_dt_dtp_pkf_base::Incubator_i_Instance_dt_dtp_pkf_base() : Node("Incubator_i_Instance_dt_dtp_pkf")
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscription_options_.callback_group = cb_group_;

    // Setting up connections
    Incubator_i_Instance_dt_dtp_pkf_device_state_subscription_ = this->create_subscription<incubator_cpp_pkg_interfaces::msg::DeviceStatei>(
        "Incubator_i_Instance_dt_dtp_pkf_device_state",
        1,
        std::bind(&Incubator_i_Instance_dt_dtp_pkf_base::handle_device_state, this, std::placeholders::_1), subscription_options_);

    Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_1 = this->create_publisher<incubator_cpp_pkg_interfaces::msg::KalmanPredictioni>(
        "Incubator_i_Instance_dt_dtp_loam_kalman_prediction",
        1);

    Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_2 = this->create_publisher<incubator_cpp_pkg_interfaces::msg::KalmanPredictioni>(
        "Incubator_i_Instance_dt_dtp_idbr_kalman_prediction",
        1);

}

//=================================================
//  C o m m u n i c a t i o n
//=================================================

void Incubator_i_Instance_dt_dtp_pkf_base::put_kalman_prediction(incubator_cpp_pkg_interfaces::msg::KalmanPredictioni msg)
{
    Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_1->publish(msg);
    Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_2->publish(msg);
}

