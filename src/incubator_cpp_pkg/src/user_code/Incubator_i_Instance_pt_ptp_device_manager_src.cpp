#include "incubator_cpp_pkg/user_headers/Incubator_i_Instance_pt_ptp_device_manager_src.hpp"
#include <ctime>
//=================================================
//  I n i t i a l i z e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_pt_ptp_device_manager::initialize()
{
    PRINT_INFO("Initialize Entry Point invoked");

    // Initialize the node
    this->declare_parameter("heater_pin", 12);
    this->declare_parameter("fan_pin", 13);
    this->declare_parameter("t1_path", "/sys/bus/w1/devices/28-0623c31c3ad6/w1_slave");
    this->declare_parameter("t2_path", "/sys/bus/w1/devices/28-0923b0b407a7/w1_slave");
    this->declare_parameter("t3_path", "/sys/bus/w1/devices/28-0823c08d9067/w1_slave");

    h = Heater(this->get_parameter("heater_pin").as_int());
    f = Fan(this->get_parameter("fan_pin").as_int());
    t1 = Thermometer(this->get_parameter("t1_path").as_string());
    t2 = Thermometer(this->get_parameter("t2_path").as_string());
    t3 = Thermometer(this->get_parameter("t3_path").as_string());

    sensorReadPeriod = 3; //TODO: Assign this to a variable during codegen so it may be obtained programatically
    timeStart = (unsigned long)time(NULL);
}

//=================================================
//  C o m p u t e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_pt_ptp_device_manager::timeTriggered()
{
    // Handle communication
    bool commandFanOn = get_request_fan_on()->data;
    bool commandHeaterOn = get_request_heater_on()->data;
    

    if(commandFanOn && !f.getState()) {
      f.ON();
    }
    else if (!commandFanOn && f.getState()) {
      f.OFF();
    }

    if(commandHeaterOn && !h.getState()) {
      h.ON();
    }
    else if(!commandHeaterOn && h.getState()) {
      h.OFF();
    }

    auto msg = incubator_cpp_pkg_interfaces::msg::DeviceStatei();

    msg.t1_time.value.data = (unsigned long)time(NULL);
    msg.t1.value.data = t1.read();
    msg.t2_time.value.data = (unsigned long)time(NULL);
    msg.t2.value.data = t2.read();
    msg.t3_time.value.data = (unsigned long)time(NULL);
    msg.t3.value.data = t3.read();
    msg.average_internal_temp.value.data = (msg.t1.value.data + msg.t2.value.data)/2.0;
    msg.heater_on.data = h.getState();
    msg.fan_on.data = f.getState();
    msg.execution_interval.value.data = sensorReadPeriod;
    msg.elapsed_time.value.data = ((unsigned long)time(NULL)) - timeStart;
    put_device_state(msg);
}

