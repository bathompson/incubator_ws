#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_pt_ptp_controller_base_src.hpp"
#include "incubator_cpp_pkg/user_headers/Controller_Model_SM.hpp"

class Incubator_i_Instance_pt_ptp_controller : public Incubator_i_Instance_pt_ptp_controller_base
{
public:
    Incubator_i_Instance_pt_ptp_controller();

private:
    //=================================================
    //  I n i t i a l i z e    E n t r y    P o i n t
    //=================================================
    void initialize();

    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    void handle_device_state(const incubator_cpp_pkg_interfaces::msg::DeviceStatei::SharedPtr msg);
    void handle_param_updates(const incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi::SharedPtr msg);

    //=================================================
    //  Include any additional declarations here
    //=================================================
    float desired_temp;
    float lower_bound;
    unsigned long heating_time;
    unsigned long heating_gap;
    float box_air_temp;
    float room_temp;
    bool heater_ctrl;
    bool fan_ctrl;
    Controller_Model_SM sm;
};
