# To Test
---
To turn fan on, run `ros2 topic pub -1 /FanOn incubator_interfaces/msg/DeviceCommand {isOn: true}`
to turn the fan off, run `ros2 topic pub -1 /FanOn incubator_interfaces/msg/DeviceCommand {isOn: false}`
To turn the heatbed on, run `ros2 topic pub -1 /HeaterOn incubator_interfaces/msg/DeviceCommand {isOn: true}`
To turn the heatbed off, run `ros2 topic pub -1 /HeaterOn incubator_interfaces/msg/DeviceCommand {isOn: false}`