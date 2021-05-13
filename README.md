# Clover Core Libraries support for ROS2 

This is a ROS2 port of the libraries written by CopterExpress for use with the Clover Drone Platform. In particular the following:

- The `clover` library from their [clover project](https://github.com/CopterExpress/clover)
- The `ros_led` project [ros_led](https://github.com/CopterExpress/ros_led). 

## Packages

- **clover_ros2**: a selection of the core clover libraries ported to ROS2
- **led_msgs**: the led_msgs from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **led_msgs_test**: Python ROS2 testing for led_msgs from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **ws281x**: led drivers ros2 nodes from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2 

## Clover ROS2

[In PROGRESS]


## LED Control

### Custom LED ROS messages (led_msgs package)

This is a port of the led_msgs ros1 package written by CopterExpress. See the original [files here](https://github.com/CopterExpress/ros_led/tree/master/led_msgs). 

This is organised as an `ament_cpp` project so the python test file had to be moved to a separate ROS package. 

> Note: custom msg/srv/actions cannot currently be included in `ament_python` build project (13/05/2021)

### WS281x LED Strip Support for ROS2 (ws281x package)

This is a port of the WS281x ROS1 package written by CopterExpress. See the original [ros1 here](https://github.com/CopterExpress/ros_led/tree/master/ws281x)

This is the LED strip used on the Clover Drone platform

This package contains the WS281x LED strip driver for ROS (on a rapsberry pi). Based on the [rpi_ws281x library](https://github.com/jgarff/rpi_ws281x)

## License

As per the original Clover source code, this ROS2 port is also covered under the MIT License.