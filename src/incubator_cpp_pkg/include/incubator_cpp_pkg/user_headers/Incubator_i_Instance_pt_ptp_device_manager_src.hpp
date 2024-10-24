#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_pt_ptp_device_manager_base_src.hpp"
#include "incubator_cpp_pkg/user_headers/Device_Layer.hpp"
class Incubator_i_Instance_pt_ptp_device_manager : public Incubator_i_Instance_pt_ptp_device_manager_base
{
public:
    Incubator_i_Instance_pt_ptp_device_manager();

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
};
