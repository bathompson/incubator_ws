#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_pt_ptp_device_manager_base_src.hpp"
#include "incubator_cpp_pkg/user_headers/Device_Layer.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/illuminance.hpp"

class Incubator_i_Instance_pt_ptp_device_manager : public Incubator_i_Instance_pt_ptp_device_manager_base
{
public:
    Incubator_i_Instance_pt_ptp_device_manager();

    void takeDownDevices()
    {
        h.OFF();
        f.OFF();
    }

private:
    //=================================================
    //  I n i t i a l i z e    E n t r y    P o i n t
    //=================================================
    void initialize();

    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    void timeTriggered();

    //=================================================
    //  Include any additional declarations here
    //=================================================
    Heater h;
    Fan f;
    Thermometer t1;
    Thermometer t2;
    Thermometer t3;
    unsigned long timeStart;
    unsigned int sensorReadPeriod;
    bool commandHeaterOn;
    bool commandFanOn;
    void convertAndSendRosTempSensorMsg(incubator_cpp_pkg_interfaces::msg::DeviceStatei msg);
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr t1DataPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr t2DataPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr t3DataPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr heatBedStatusPublisher;
};
