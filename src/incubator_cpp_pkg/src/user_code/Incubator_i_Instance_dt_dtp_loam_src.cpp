#include "incubator_cpp_pkg/user_headers/Incubator_i_Instance_dt_dtp_loam_src.hpp"
#include <cmath>

//=================================================
//  I n i t i a l i z e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_dt_dtp_loam::initialize()
{
    PRINT_INFO("Initialize Entry Point invoked");
    this->declare_parameter("target-temp", 35.0f);
    this->declare_parameter("error-threshold", 1.0f);

    // Initialize the node
    desired_target_temperature = cur_target_temp = this->get_parameter("target-temp").as_double();
    error_threshold = this->get_parameter("error-threshold").as_double();
}

//=================================================
//  C o m p u t e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_dt_dtp_loam::handle_kalman_prediction(const incubator_cpp_pkg_interfaces::msg::KalmanPredictioni::SharedPtr msg)
{
    // Handle kalman_prediction msg
    float new_target_temp = std::abs(msg->prediction_error.value.data) > error_threshold ? 0.6*desired_target_temperature : desired_target_temperature;
    if(new_target_temp != cur_target_temp)
    {
        auto paramUpdateMsg = incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi();
        paramUpdateMsg.target_temperature.value.data = new_target_temp;
        put_param_updates(paramUpdateMsg);
        cur_target_temp = new_target_temp;
    }
}

