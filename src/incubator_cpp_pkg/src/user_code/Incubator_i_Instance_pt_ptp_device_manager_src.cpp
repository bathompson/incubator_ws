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

    t1DataPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("t1_temp", 10);
    t2DataPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("t2_temp", 10);
    t3DataPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("t3_temp", 10);
    heatBedStatusPublisher = this->create_publisher<sensor_msgs::msg::Illuminance>("heat_bed_status", 10);

    auto heaterPin = this->get_parameter("heater_pin").as_int();
    auto fanPin = this->get_parameter("fan_pin").as_int();

    auto t1Path = this->get_parameter("t1_path").as_string();
    auto t2Path = this->get_parameter("t2_path").as_string();
    auto t3Path = this->get_parameter("t3_path").as_string();

    h = Heater(heaterPin);
    f = Fan(fanPin);
    t1 = Thermometer(t1Path);
    t2 = Thermometer(t2Path);
    t3 = Thermometer(t3Path);

    sensorReadPeriod = 3; //TODO: Assign this to a variable during codegen so it may be obtained programatically
    timeStart = (unsigned long)time(NULL);
}

//=================================================
//  C o m p u t e    E n t r y    P o i n t
//=================================================
void Incubator_i_Instance_pt_ptp_device_manager::timeTriggered()
{
    // Handle communication
    bool commandFanOn = false;
    bool commandHeaterOn = false;
    auto requestFanOnMsg = get_request_fan_on();
    auto requestHeaterOnMsg = get_request_heater_on();
    if(requestFanOnMsg)
    {
      commandFanOn = requestFanOnMsg->data;
    }
    if(requestHeaterOnMsg)
    {
      commandHeaterOn = requestHeaterOnMsg->data;
    }

    
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
    convertAndSendRosTempSensorMsg(msg);
}

void Incubator_i_Instance_pt_ptp_device_manager::convertAndSendRosTempSensorMsg(incubator_cpp_pkg_interfaces::msg::DeviceStatei msg)
{
  auto t1Msg = sensor_msgs::msg::Temperature();
  auto t2Msg = sensor_msgs::msg::Temperature();
  auto t3Msg = sensor_msgs::msg::Temperature();

  auto heatBedMsg = sensor_msgs::msg::Illuminance();

  t1Msg.header.stamp.sec = msg.t1_time.value.data;
  t1Msg.header.stamp.nanosec = 0;
  t1Msg.header.frame_id = "t1";
  t1Msg.temperature = msg.t1.value.data;
  t1Msg.variance = 0;

  t2Msg.header.stamp.sec = msg.t2_time.value.data;
  t2Msg.header.stamp.nanosec = 0;
  t2Msg.header.frame_id = "t2";
  t2Msg.temperature = msg.t2.value.data;
  t2Msg.variance = 0;

  t3Msg.header.stamp.sec = msg.t3_time.value.data;
  t3Msg.header.stamp.nanosec = 0;
  t3Msg.header.frame_id = "t3";
  t3Msg.temperature = msg.t3.value.data;
  t3Msg.variance = 0;

  heatBedMsg.header.stamp.sec = msg.t3_time.value.data;
  heatBedMsg.header.stamp.nanosec = 0;
  heatBedMsg.header.frame_id = "heat_bed";
  if(msg.heater_on.data)
    heatBedMsg.illuminance = 1000.0;
  else
    heatBedMsg.illuminance = 0.0;
  heatBedMsg.variance = 0;

  t1DataPublisher->publish(t1Msg);
  t2DataPublisher->publish(t2Msg);
  t3DataPublisher->publish(t3Msg);
  heatBedStatusPublisher->publish(heatBedMsg);
}

