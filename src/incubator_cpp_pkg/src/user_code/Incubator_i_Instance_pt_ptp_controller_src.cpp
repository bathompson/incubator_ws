#include "incubator_cpp_pkg/user_headers/Incubator_i_Instance_pt_ptp_controller_src.hpp"
#include "incubator_cpp_pkg/user_headers/util.hpp"
//=================================================
//  I n i t i a l i z e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_pt_ptp_controller::initialize()
{
    PRINT_INFO("Initialize Entry Point invoked");

    // Initialize the node
    this->declare_parameter("desired_temp", 35.0f);
    this->declare_parameter("lower_bound", 5.0f);
    this->declare_parameter("heating_time", 20);
    this->declare_parameter("heating_gap", 30);
    desired_temp = this->get_parameter("desired_temp").as_double();
    lower_bound = this->get_parameter("lower_bound").as_double();
    heating_time = this->get_parameter("heating_time").as_int();
    heating_gap = this->get_parameter("heating_gap").as_int();
    sm = Controller_Model_SM(desired_temp, lower_bound, heating_time, heating_gap);
}

//=================================================
//  C o m p u t e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_pt_ptp_controller::handle_device_state(const incubator_cpp_pkg_interfaces::msg::DeviceStatei::SharedPtr msg)
{
    // Handle device_state msg
    // Record sensor data
    PRINT_INFO("Internal box temp is %.02f. Room temp is %.02f", msg->average_internal_temp.value.data, msg->t3.value.data);
    box_air_temp = msg->average_internal_temp.value.data;
    room_temp = msg->t3.value.data;

    if(box_air_temp >= 58) {
        RCLCPP_ERROR(this->get_logger(), "Temperature exceeds 58, cleaning up...");
        rclcpp::shutdown();
        exit(1);
    }

    fan_ctrl = true;
    sm.step((unsigned long)time(NULL), box_air_temp);
    heater_ctrl = sm.cached_heater_on;

    auto heaterControlMsg = incubator_cpp_pkg_interfaces::msg::Boolean();
    heaterControlMsg.data = heater_ctrl;
    auto fanControlMsg = incubator_cpp_pkg_interfaces::msg::Boolean();
    fanControlMsg.data = fan_ctrl;
    put_request_heater_on(heaterControlMsg);
    put_request_fan_on(fanControlMsg);

    auto controllerStatusMsg = incubator_cpp_pkg_interfaces::msg::ControllerStatusi();

    controllerStatusMsg.cur_time.value.data = (unsigned long)time(NULL);
    controllerStatusMsg.heater_on.data = heater_ctrl;
    controllerStatusMsg.fan_on.data = fan_ctrl;
    controllerStatusMsg.current_state.controller_state = convertIncubatorStateToRosEnum(sm.cur_state);
    controllerStatusMsg.next_time.value.data = sm.nextTime ? *sm.nextTime : -1;
    controllerStatusMsg.target_temp.value.data = sm.desired_temp;
    controllerStatusMsg.lower_bound.value.data = sm.lower_bound;
    controllerStatusMsg.heating_time.value.data = sm.heating_time;
    controllerStatusMsg.heating_gap.value.data = sm.heating_gap;

    PRINT_INFO("cur_time: %lu, heater_on: %s, fan_on: %s, controller_state: %s, next_time: %lu, target_temp: %.02f, lower_bound: %.02f, heating_time: %lu, heating_gap: %lu",
                controllerStatusMsg.cur_time.value.data, controllerStatusMsg.heater_on.data == true ? "true" : "false", controllerStatusMsg.fan_on.data == true ? "true" : "false",
                convertIncubatorStateRosEnumToString(controllerStatusMsg.current_state.controller_state).c_str(), controllerStatusMsg.next_time.value.data, 
                controllerStatusMsg.target_temp.value.data, controllerStatusMsg.lower_bound.value.data, controllerStatusMsg.heating_time.value.data, controllerStatusMsg.heating_gap.value.data);

    put_controller_status(controllerStatusMsg);

}

void Incubator_i_Instance_pt_ptp_controller::handle_param_updates(const incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi::SharedPtr msg)
{
    // Handle param_updates msg
    RCLCPP_INFO(this->get_logger(), "Setting param updates");
    sm.desired_temp = msg->target_temperature.value.data;
    desired_temp = sm.desired_temp;
}

