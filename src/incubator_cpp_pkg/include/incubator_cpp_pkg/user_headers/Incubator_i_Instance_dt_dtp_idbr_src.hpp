#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_dt_dtp_idbr_base_src.hpp"
#include "incubator_cpp_pkg/user_headers/influxdb.hpp"

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
    void handle_kalman_prediction(const incubator_cpp_pkg_interfaces::msg::KalmanPredictioni::SharedPtr msg);
    void handle_param_updates(const incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi::SharedPtr msg);

    //=================================================
    //  Include any additional declarations here
    //=================================================
    std::shared_ptr<influxdb_cpp::server_info> si;

};
