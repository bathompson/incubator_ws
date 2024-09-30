#include "rclcpp/rclcpp.hpp"   // rclcpp must be declared as a dependency in package.xml and CMakeLists.txt

// ------ example interface import ------
// include interfaces for message type
#include "incubator_interfaces/msg/device_command.hpp"
#include "incubator_interfaces/msg/device_sensor_state.hpp"
#include "PhysicalTwin_cpp/Controller_Model_SM.hpp"
#include <ctime>

class Controller_Node : public rclcpp::Node // Rename class as appropriate
{

public: // Declares everything that follows as public
  
  Controller_Node() : Node("controller_node"), desired_temp(35.0), lower_bound(5.0), heating_time(20), heating_gap(30), sm(desired_temp, lower_bound, heating_time, heating_gap) // The name on the left should match the class name, and the name in quotes is the *run-time ROS2 node object*
  {

    // ----- example publisher set up -------------

    // create a publisher for topic named OutTopic (appears as /OutTopic in ROS2 queries)
    //   ..with message type Int32 and output queue size of 10
    HeaterOnPublisher = this->create_publisher<incubator_interfaces::msg::DeviceCommand>("HeaterOn", 10);
    FanOnPublisher = this->create_publisher<incubator_interfaces::msg::DeviceCommand>("FanOn", 10);

    // example code to publish on topic
    //  auto msg = example_interfaces::msg::Int32();   // create message data structure ("auto" automatically assigns the type of msg)
    //  msg.data = <some int value>;                   // assign value to payload field
    //  myOutTopicPublisher_->publish(msg);            // send out message data structure

    // ----- example subscriber set up -------------

    // create a subscriber for topic named InTopic (appears as /InTopic in ROS2 queries)
    //   ..with message type Int32 and associated handler method for messages 
    SensorDataSubscriber = this->create_subscription<incubator_interfaces::msg::DeviceSensorState>(
      "SensorData", // topic name
      10, // buffer size
      std::bind(&Controller_Node::handle_InSensorData, this, std::placeholders::_1) // name of callback function
                                                                           // (the std::placeholders::_1 acts as a placeholder for the message parameter)
    );

    RCLCPP_INFO(this->get_logger(), "My_Cpp_Node infrastructure set up"); // Rename as necessary

    //--------------- user initialization code ---------------
    
    // invoke initialize entry point
    initialize();
  }

private: // Declares everything that follows as private

  //================================================= 
  //-------------------------------------------------
  //  A p p l i c a t i o n     C o d e 
  //-------------------------------------------------
  //================================================= 

  //================================================= 
  //  A p p    I n i t i a l i z e 
  //=================================================    
  void initialize() {
    RCLCPP_INFO(this->get_logger(), "Initialize Entry Point invoked");
  }

  //================================================= 
  //  A p p    handle_inTopic
  //
  //     Example handler for input topic
  //=================================================  

  void handle_InSensorData(incubator_interfaces::msg::DeviceSensorState msg) {
    recordSensorData(msg);

    ctrl_step();

    print_device_state(msg);
  }

  void print_device_state(incubator_interfaces::msg::DeviceSensorState msg) {
    RCLCPP_INFO(this->get_logger(), "time=%lu, exec_interval=%d, elapsed=%lu, heater_on=%s, fan_on=%s, room_temp=%.02f, box_temp=%.02f, state=%s", msg.timestamp, msg.execution_interval, msg.elapsed_time, msg.is_heater_on ? "On" : "Off", msg.is_fan_on?"On":"Off", msg.internal_temp, msg.t3, print_state(sm.cur_state));
  }

  const char *print_state(IncubatorState state) {
    switch(state) {
      case IncubatorState::CoolingDown:
        return "CoolingDown";
      case IncubatorState::Heating:
        return "Heating";
      case IncubatorState::Waiting:
        return "Waiting";
      default:
        return "Invalid State";
    }
  }


  void recordSensorData(incubator_interfaces::msg::DeviceSensorState state) {
    box_air_temp = state.internal_temp;
    room_temp = state.t3;
  }

  void ctrl_step() {
    if(box_air_temp >= 58) {
      RCLCPP_ERROR(this->get_logger(), "Temperature exceeds 58, Cleaning up");
      rclcpp::shutdown();
      exit(1);
    }
    fan_ctrl = true;
    sm.step((unsigned long)time(NULL), box_air_temp);
    heater_ctrl = sm.cached_heater_on;
  }

  void send_commands() {
    auto heaterControlMsg = incubator_interfaces::msg::DeviceCommand();
    heaterControlMsg.is_on = heater_ctrl;
    HeaterOnPublisher->publish(heaterControlMsg);
    auto fanControlMsg = incubator_interfaces::msg::DeviceCommand();
    fanControlMsg.is_on = fan_ctrl;
    FanOnPublisher->publish(fanControlMsg);
  }
    
  //================================================= 
  //  I n s t a n c e   V a r i a b l e s
  //================================================= 
  float desired_temp;
  float lower_bound;
  unsigned long heating_time;
  unsigned long heating_gap;
  float box_air_temp;
  float room_temp;
  bool heater_ctrl;
  bool fan_ctrl;
  rclcpp::Publisher<incubator_interfaces::msg::DeviceCommand>::SharedPtr HeaterOnPublisher;
  rclcpp::Publisher<incubator_interfaces::msg::DeviceCommand>::SharedPtr FanOnPublisher;
  rclcpp::Subscription<incubator_interfaces::msg::DeviceSensorState>::SharedPtr SensorDataSubscriber;
  Controller_Model_SM sm;
};
    
//================================================= 
//-------------------------------------------------
//  I n f r a s t r u c t u r e     C o d e 
//    for Python node file
//-------------------------------------------------
//================================================= 


// Infrastructure entry point
//   - you will need to add a reference to this in setup.py

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                     // start ROS2 infrastructure 
    auto node = std::make_shared<Controller_Node>();  // create ROS2 node - RENAME class to definition above
    rclcpp::spin(node);                           // start ROS2 repeatedly execute callbacks for timers and incoming messages
    rclcpp::shutdown();                           // shutdown ROS2 infrastructure
    return 0;
}
