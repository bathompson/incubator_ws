#!/usr/bin/env python3
import rclpy   # rclpy must be declared as a dependence in package.xml - this should be added automatically when you create the package
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from incubator_cpp_pkg_interfaces.msg import DeviceStatei, KalmanPredictioni
from incubator_py_pkg.Kalman_Filter_4P import construct_filter
import numpy as np

class Incubator_i_Instance_dt_dtp_pkf(Node):   # Rename class as appropriate
   
  def __init__(self):

    # Node Class infrastructure

    super().__init__("Incubator_i_Instance_dt_dtp_pkf")    # Rename to name of *run-time ROS2 node object* 
        
    self.cb_group_ = ReentrantCallbackGroup()
    
    self.Incubator_i_Instance_dt_dtp_pkf_device_state_subscription_ = self.create_subscription(
       DeviceStatei,
       "Incubator_i_Instance_dt_dtp_pkf_device_state",
       self.handle_device_state,
       1,
       callback_group=self.cb_group_
    )

    self.Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_1 = self.create_publisher(
       KalmanPredictioni,
       "Incubator_i_Instance_dt_dtp_loam_kalman_prediction",
       1
    )

    self.Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_2 = self.create_publisher(
       KalmanPredictioni,
       "Incubator_i_Instance_dt_dtp_idbr_kalman_prediction",
       1
    )

    self.filter = None
    self.in_heater = None
    self.in_room_T = None
    self.in_T = None
    self.step_size = None
    self.T_heater = None
    self.std_dev = None
    self.Theater_covariance_init = None
    self.T_covariance_init = None
    # invoke initialize entry point
    self.initialize()

  #================================================= 
  #  A p p    I n i t i a l i z e 
  #=================================================    
  def put_kalman_prediction(self, msg: KalmanPredictioni):
     self.Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_1.publish(msg)
     self.Incubator_i_Instance_dt_dtp_pkf_kalman_prediction_publisher_2.publish(msg)

  def handle_device_state(self, msg: DeviceStatei):
     self.in_heater = 1.0 if msg.heater_on.data else 0.0
     self.in_room_T = msg.t3.value.data

     self.in_T = msg.average_internal_temp.value.data

     self.filter.predict(u=np.array([
        [self.in_heater],
        [self.in_room_T]
     ]))
     self.filter.update(np.array([[self.in_T]]))

     next_x = self.filter.x
     self.T_heater = next_x[0, 0]
     T = next_x[1, 0]

     kPredMsg = KalmanPredictioni()

     kPredMsg.t_heater.value.data = self.T_heater
     kPredMsg.prediction_error.value.data = self.in_T - T
     kPredMsg.prediction_time.value.data = int(time.time())

     self.put_kalman_prediction(kPredMsg)

  def initialize (self):
    self.get_logger().info("Initialize Entry Point invoked")
    

    self.declare_parameter("step-size", 3)
    self.declare_parameter("std-dev", 0.001)
    self.declare_parameter("Theater-covariance-init", 0.001)
    self.declare_parameter("T-covariance-init", 0.001)
    self.declare_parameter("C-air", 267.55929458)
    self.declare_parameter("G-box", 0.5763498)
    self.declare_parameter("C-heater", 329.25376821)
    self.declare_parameter("G-heater", 1.67053237)
    self.declare_parameter("V-heater", 12.15579391)
    self.declare_parameter("I-heater", 1.53551347)
    self.declare_parameter("initial-heat-temperature", 21.0)
    self.declare_parameter("initial-box-temperature", 21.0)
    
    self.step_size = self.get_parameter("step-size").get_parameter_value().integer_value
    self.std_dev = self.get_parameter("std-dev").get_parameter_value().double_value
    self.Theater_covariance_init = self.get_parameter("Theater-covariance-init").get_parameter_value().double_value
    self.T_covariance_init = self.get_parameter("T-covariance-init").get_parameter_value().double_value

    self.filter = construct_filter(
       self.step_size, self.std_dev, self.Theater_covariance_init, self.T_covariance_init,
       self.get_parameter("C-air").get_parameter_value().double_value,
       self.get_parameter("G-box").get_parameter_value().double_value,
       self.get_parameter("C-heater").get_parameter_value().double_value,
       self.get_parameter("G-heater").get_parameter_value().double_value,
       self.get_parameter("V-heater").get_parameter_value().double_value,
       self.get_parameter("I-heater").get_parameter_value().double_value,
       self.get_parameter("initial-heat-temperature").get_parameter_value().double_value,
       self.get_parameter("initial-box-temperature").get_parameter_value().double_value
    )

    



# end Class

#================================================= 
#-------------------------------------------------
#  I n f r a s t r u c t u r e     C o d e 
#    for Python node file
#-------------------------------------------------
#================================================= 


# Infrastructure entry point
#   - you will need to add a reference to this in setup.py

def main(args=None):
    rclpy.init(args=args)  # start ROS2 infrastructure 
    executor = MultiThreadedExecutor()
    node = Incubator_i_Instance_dt_dtp_pkf()    # create ROS2 node - RENAME class to definition above
    executor.add_node(node)
    executor.spin()       # start ROS2 repeatedly execute callbacks for timers and incoming messages
    rclpy.shutdown()       # shutdown ROS2 infrastructure


# Required if you want to run the python file directly. BUT 
#  we will almost always call it via the setup.py file and ros2 run node or launch.
#  It is not needed in those situations.
if __name__ == "__main__":
    main()