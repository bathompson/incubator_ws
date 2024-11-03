#include "incubator_cpp_pkg/base_headers/Incubator_i_Instance_dt_dtp_loam_base_src.hpp"

class Incubator_i_Instance_dt_dtp_loam : public Incubator_i_Instance_dt_dtp_loam_base
{
public:
    Incubator_i_Instance_dt_dtp_loam();

private:
    //=================================================
    //  I n i t i a l i z e    E n t r y    P o i n t
    //=================================================
    void initialize();

    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    void handle_kalman_prediction(const incubator_cpp_pkg_interfaces::msg::KalmanPredictioni::SharedPtr msg);

    //=================================================
    //  Include any additional declarations here
    //=================================================
    float desired_target_temperature;
    float error_threshold;
    float cur_target_temp;
};
