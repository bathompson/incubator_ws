#include "rclcpp/rclcpp.hpp"   // rclcpp must be declared as a dependency in package.xml and CMakeLists.txt

// ------ example interface import ------
// include interfaces for message type
#include "incubator_interfaces/msg/device_command.hpp"
#include "incubator_interfaces/msg/device_sensor_state.hpp"
#include "PhysicalTwin_cpp/Device_Layer.hpp"
#include <pigpio.h>
#include <ctime>
#include <string>
#include <set>
#include <stdexcept>
#include <memory>
#include <iostream>
#include <fstream>
#include <regex>
#include <limits>
    
class Sensor_Driver_Node : public rclcpp::Node // Rename class as appropriate
{

public: // Declares everything that follows as public
  
  Sensor_Driver_Node() : Node("sensor_driver"), h(12), f(13), 
  t1(std::make_shared<std::string>("/sys/bus/w1/devices/28-0623c31c3ad6/w1_slave")), 
  t2(std::make_shared<std::string>("/sys/bus/w1/devices/28-0923b0b407a7/w1_slave")),
  t3(std::make_shared<std::string>("/sys/bus/w1/devices/28-0823c08d9067/w1_slave"))
  // The name on the left should match the class name, and the name in quotes is the *run-time ROS2 node object*
  {
    // ----- example timer set up -------------

    sensorReadPeriod = 3; // the "period" in SECONDS of timer invocation; i.e., time between each timer invocation
    // create a timer with the indicated period and associated handler method for the timer events
    // store a reference to the timer in the instance field myTimer_
    timeStart = (unsigned long) time(NULL);

    deviceUpdateTimer = this->create_wall_timer(std::chrono::seconds(sensorReadPeriod), 
      std::bind(&Sensor_Driver_Node::handle_device_update, this));

    // ----- example publisher set up -------------

    // create a publisher for topic named OutTopic (appears as /OutTopic in ROS2 queries)
    //   ..with message type Int32 and output queue size of 10
    SensorDataPublisher = this->create_publisher<incubator_interfaces::msg::DeviceSensorState>("SensorData", 10);

    // example code to publish on topic
    //  auto msg = example_interfaces::msg::Int32();   // create message data structure ("auto" automatically assigns the type of msg)
    //  msg.data = <some int value>;                   // assign value to payload field
    //  myOutTopicPublisher_->publish(msg);            // send out message data structure

    // ----- example subscriber set up -------------

    // create a subscriber for topic named InTopic (appears as /InTopic in ROS2 queries)
    //   ..with message type Int32 and associated handler method for messages 
    HeaterOnSubscriber = this->create_subscription<incubator_interfaces::msg::DeviceCommand>(
      "HeaterOn", // topic name
      10, // buffer size
      std::bind(&Sensor_Driver_Node::handle_HeaterOn_msg, this, std::placeholders::_1)); // name of callback function
                                                                           // (the std::placeholders::_1 acts as a placeholder for the message parameter)
    FanOnSubscriber = this->create_subscription<incubator_interfaces::msg::DeviceCommand>(
      "FanOn",
      10,
      std::bind(&Sensor_Driver_Node::handle_FanOn_msg, this, std::placeholders::_1)
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

    // Initialize local state vars
    commandHeaterOn = false;
    commandFanOn = false;
  }

  //================================================= 
  //  A p p    handle_myTimer
  //
  //     Example handler for timer
  //=================================================  

  void handle_device_update() {   // copy and rename as appropriate for each timer
    update_actuators();
    read_and_publish_state();
  }

  void update_actuators() {
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
  }

  void read_and_publish_state() {
    auto msg = incubator_interfaces::msg::DeviceSensorState();
    msg.time_t1 = (unsigned long)time(NULL);
    msg.t1 = t1.read();
    msg.time_t2 = (unsigned long)time(NULL);
    msg.t2 = t2.read();
    msg.time_t3 = (unsigned long)time(NULL);
    msg.t3 = t3.read();
    msg.internal_temp = (msg.t1+msg.t2)/2.0;
    msg.is_heater_on = h.getState();
    msg.is_fan_on = f.getState();
    msg.execution_interval = sensorReadPeriod;
    msg.timestamp = (unsigned long)time(NULL);
    msg.elapsed_time = msg.timestamp - timeStart;
    print_state(msg);
    SensorDataPublisher->publish(msg);
  }

  void print_state(incubator_interfaces::msg::DeviceSensorState state) {
    RCLCPP_INFO(this->get_logger(), "elapsed_time = %lu; t1 = %.02f; t2 = %.02f; t3 = %.02f; internal_temp = %.02f; is_heater_on = %s; is_fan_on = %s", state.elapsed_time, state.t1, state.t2, state.t3, state.internal_temp, state.is_heater_on ? "true" : "false", state.is_fan_on ? "true" : "false");
  }

  //================================================= 
  //  A p p    handle_inTopic
  //
  //     Example handler for input topic
  //=================================================  

  void handle_HeaterOn_msg(incubator_interfaces::msg::DeviceCommand msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: %s", msg.is_on ? "heater on" : "heater off");  // Echo incoming data
    commandHeaterOn = msg.is_on;
  }

  void handle_FanOn_msg(incubator_interfaces::msg::DeviceCommand msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: %s", msg.is_on ? "fan on" : "fan off");  // Echo incoming data
    commandFanOn = msg.is_on;
  }
    
  //================================================= 
  //  I n s t a n c e   V a r i a b l e s
  //================================================= 
  rclcpp::Publisher<incubator_interfaces::msg::DeviceSensorState>::SharedPtr SensorDataPublisher;
  rclcpp::Subscription<incubator_interfaces::msg::DeviceCommand>::SharedPtr HeaterOnSubscriber;
  rclcpp::Subscription<incubator_interfaces::msg::DeviceCommand>::SharedPtr FanOnSubscriber;
  rclcpp::TimerBase::SharedPtr deviceUpdateTimer;
  bool commandHeaterOn;
  bool commandFanOn;

  Heater h;
  Fan f;
  Thermometer t1;
  Thermometer t2;
  Thermometer t3;
  unsigned long timeStart;
  unsigned int sensorReadPeriod;
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
  if(gpioInitialise() < 0) return 1;
  rclcpp::init(argc, argv);                     // start ROS2 infrastructure 
  auto node = std::make_shared<Sensor_Driver_Node>();  // create ROS2 node - RENAME class to definition above
  rclcpp::spin(node);                           // start ROS2 repeatedly execute callbacks for timers and incoming messages
  rclcpp::shutdown();                           // shutdown ROS2 infrastructure
  gpioTerminate();
  return 0;
}
