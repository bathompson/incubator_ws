#include "rclcpp/rclcpp.hpp"   // rclcpp must be declared as a dependency in package.xml and CMakeLists.txt
#include "incubator_interfaces/msg/device_sensor_state.hpp"
#include <InfluxDBFactory.h>

class InfluxDB_Recorder_Node : public rclcpp::Node // Rename class as appropriate
{

public: // Declares everything that follows as public
  
  InfluxDB_Recorder_Node() : Node("influxdb_recorder_node") // The name on the left should match the class name, and the name in quotes is the *run-time ROS2 node object*
  {
    // ----- example subscriber set up -------------

    // create a subscriber for topic named InTopic (appears as /InTopic in ROS2 queries)
    //   ..with message type Int32 and associated handler method for messages 
    deviceSensorStateSubscriber = this->create_subscription<incubator_interfaces::msg::DeviceSensorState>(
      "SensorData", // topic name
      10, // buffer size
      std::bind(&InfluxDB_Recorder_Node::handle_InTopic, this, std::placeholders::_1) // name of callback function
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

    // Initialize local state vars
    // this->dummy = 0;

    auto db = influxdb::InfluxDBFactory::Get("http://localhost:8086?db=incubator_db");
    db->createDatabaseIfNotExists();
    for (auto i: db->query("SHOW DATABASES")) std::cout<<i.getTags()<<std::endl;

  }

  //================================================= 
  //  A p p    handle_inTopic
  //
  //     Example handler for input topic
  //=================================================  

  void handle_InTopic(incubator_interfaces::msg::DeviceSensorState msg) {
    RCLCPP_INFO(this->get_logger(), "Handler for InTopic invoked");
  }
    
  //================================================= 
  //  I n s t a n c e   V a r i a b l e s
  //================================================= 
  rclcpp::Subscription<incubator_interfaces::msg::DeviceSensorState>::SharedPtr deviceSensorStateSubscriber;
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
    auto node = std::make_shared<InfluxDB_Recorder_Node>();  // create ROS2 node - RENAME class to definition above
    rclcpp::spin(node);                           // start ROS2 repeatedly execute callbacks for timers and incoming messages
    rclcpp::shutdown();                           // shutdown ROS2 infrastructure
    return 0;
}