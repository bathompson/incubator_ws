#include "incubator_cpp_pkg/user_headers/Incubator_i_Instance_dt_dtp_idbr_src.hpp"
#include "incubator_cpp_pkg/user_headers/util.hpp"
#include <chrono>

//=================================================
//  I n i t i a l i z e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_dt_dtp_idbr::initialize()
{
    PRINT_INFO("Initialize Entry Point invoked");
    this->declare_parameter("influxdb-ip", "127.0.0.1");
    this->declare_parameter("influxdb-port", 8086);
    this->declare_parameter("influxdb-org", "incubator");
    this->declare_parameter("influxdb-token", "TWg1Rtq_RyuJpCbxZmFwMfSj7DJeVqkAcY_jPO-c3PAueGrYIPFIaK2sEFCpvG0NVZsQpBuglS_3Ziwo6SDv6Q==");
    this->declare_parameter("influxdb-bucket", "incubator");

    auto ip = this->get_parameter("influxdb-ip").as_string();
    int port = this->get_parameter("influxdb-port").as_int();
    auto org = this->get_parameter("influxdb-org").as_string();
    auto token = this->get_parameter("influxdb-token").as_string();
    auto bucket = this->get_parameter("influxdb-bucket").as_string();
    // Initialize the node
    si = std::make_shared<influxdb_cpp::server_info>(ip, port, org, token, bucket);
}

//=================================================
//  C o m p u t e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_dt_dtp_idbr::handle_controller_status(const incubator_cpp_pkg_interfaces::msg::ControllerStatusi::SharedPtr msg)
{
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    
    influxdb_cpp::builder()
    .meas("pt_controller")
    .field("cur_time", msg->cur_time.value.data)
    .field("heater_on", msg->heater_on.data)
    .field("fan_on", msg->fan_on.data)
    .field("current_state", convertIncubatorStateRosEnumToString(msg->current_state.controller_state))
    .field("next_time", msg->next_time.value.data)
    .field("target_temp", msg->target_temp.value.data)
    .field("lower_bound", msg->lower_bound.value.data)
    .field("heating_time", msg->heating_time.value.data)
    .field("heating_gap", msg->heating_gap.value.data)
    .timestamp(time)
    .post_http(*si);
}

void Incubator_i_Instance_dt_dtp_idbr::handle_device_state(const incubator_cpp_pkg_interfaces::msg::DeviceStatei::SharedPtr msg)
{
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    influxdb_cpp::builder()
    .meas("device_manager")
    .field("t1_temp", msg->t1.value.data)
    .field("t1_time", msg->t1_time.value.data)
    .field("t2_temp", msg->t2.value.data)
    .field("t2_time", msg->t2_time.value.data)
    .field("t3_temp", msg->t3.value.data)
    .field("t3_time", msg->t3_time.value.data)
    .field("box_temp", msg->average_internal_temp.value.data)
    .field("heater_on_cmd", msg->heater_on.data)
    .field("fan_on_cmd", msg->fan_on.data)
    .field("execution_interval", msg->execution_interval.value.data)
    .field("elapsed_time", msg->elapsed_time.value.data)
    .timestamp(time)
    .post_http(*si);
}

void Incubator_i_Instance_dt_dtp_idbr::handle_kalman_prediction(const incubator_cpp_pkg_interfaces::msg::KalmanPredictioni::SharedPtr msg)
{
    // Handle kalman_prediction msg
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    influxdb_cpp::builder()
    .meas("kalman_filter")
    .field("t_heater", msg->t_heater.value.data)
    .field("prediction_error", msg->t_heater.value.data)
    .field("average_temperature", msg->average_temperature.value.data)
    .field("prediction_time", msg->prediction_time.value.data)
    .timestamp(time)
    .post_http(*si);
}

void Incubator_i_Instance_dt_dtp_idbr::handle_param_updates(const incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi::SharedPtr msg)
{
    // Handle param_updates msg
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    influxdb_cpp::builder()
    .meas("closed_loop_param_updates")
    .field("target_temp", msg->target_temperature.value.data)
    .timestamp(time)
    .post_http(*si);
}

