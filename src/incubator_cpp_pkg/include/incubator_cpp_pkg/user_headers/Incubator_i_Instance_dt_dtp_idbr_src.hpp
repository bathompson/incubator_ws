#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_dt_dtp_idbr_base_src.hpp"

class Incubator_i_Instance_dt_dtp_idbr : public Incubator_i_Instance_dt_dtp_idbr_base
{
public:
    Incubator_i_Instance_dt_dtp_idbr();

private:
    //=================================================
    //  I n i t i a l i z e    E n t r y    P o i n t
    //=================================================
    void initialize();

    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    void handle_controller_status(const incubator_cpp_pkg_interfaces::msg::ControllerStatusi::SharedPtr msg);
    void handle_device_state(const incubator_cpp_pkg_interfaces::msg::DeviceStatei::SharedPtr msg);

    //=================================================
    //  Include any additional declarations here
    //=================================================

};
