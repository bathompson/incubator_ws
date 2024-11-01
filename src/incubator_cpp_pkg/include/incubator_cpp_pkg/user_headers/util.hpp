#include "incubator_cpp_pkg_interfaces/msg/controller_statusi.hpp"
#include "incubator_cpp_pkg/user_headers/Controller_Model_SM.hpp"
#include <string>

uint8_t convertIncubatorStateToRosEnum(IncubatorState s) 
{
    if(s == IncubatorState::Heating) return incubator_cpp_pkg_interfaces::msg::ControllerState::CONTROLLER_STATE_HEATING;
    if(s == IncubatorState::CoolingDown) return incubator_cpp_pkg_interfaces::msg::ControllerState::CONTROLLER_STATE_COOLING;
    else return incubator_cpp_pkg_interfaces::msg::ControllerState::CONTROLLER_STATE_WAITING;

}

IncubatorState convertIncubatorStateRosEnumToIncubatorState(uint8_t s)
{
    switch(s)
    {
        case incubator_cpp_pkg_interfaces::msg::ControllerState::CONTROLLER_STATE_HEATING:
            return IncubatorState::Heating;
        case incubator_cpp_pkg_interfaces::msg::ControllerState::CONTROLLER_STATE_COOLING:
            return IncubatorState::CoolingDown;
        case incubator_cpp_pkg_interfaces::msg::ControllerState::CONTROLLER_STATE_WAITING:
        default:
            return IncubatorState::Waiting;
    }
}

std::string convertIncubatorStateToString(IncubatorState s)
{
    switch(s)
    {
        case IncubatorState::Heating:
            return "Heating";
        case IncubatorState::CoolingDown:
            return "Cooling Down";
        case IncubatorState::Waiting:
        default:
            return "Waiting";
    }
}

std::string convertIncubatorStateRosEnumToString(uint8_t s)
{
    return convertIncubatorStateToString(convertIncubatorStateRosEnumToIncubatorState(s));
}