#include "rclcpp/rclcpp.hpp"
#include "incubator_cpp_pkg_interfaces/msg/kalman_predictioni.hpp"
#include "incubator_cpp_pkg_interfaces/msg/closed_loop_param_updatesi.hpp"
#include <queue>

//=================================================
//  D O   N O T   E D I T   T H I S   F I L E
//=================================================

class Incubator_i_Instance_dt_dtp_loam_base : public rclcpp::Node
{
protected:
    Incubator_i_Instance_dt_dtp_loam_base();

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================

    #define PRINT_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
    #define PRINT_WARN(...) RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
    #define PRINT_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)

    void put_param_updates(incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi msg);

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // SubscriptionOptions for assigning subscriptions to the callback group
    rclcpp::SubscriptionOptions subscription_options_;


    //=================================================
    //  C o m p u t e    E n t r y    P o i n t
    //=================================================
    virtual void handle_kalman_prediction(const incubator_cpp_pkg_interfaces::msg::KalmanPredictioni::SharedPtr msg) = 0;

    //=================================================
    //  C o m m u n i c a t i o n
    //=================================================
    rclcpp::Subscription<incubator_cpp_pkg_interfaces::msg::KalmanPredictioni>::SharedPtr Incubator_i_Instance_dt_dtp_loam_kalman_prediction_subscription_;

    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi>::SharedPtr Incubator_i_Instance_dt_dtp_loam_param_updates_publisher_1;
    rclcpp::Publisher<incubator_cpp_pkg_interfaces::msg::ClosedLoopParamUpdatesi>::SharedPtr Incubator_i_Instance_dt_dtp_loam_param_updates_publisher_2;

};
