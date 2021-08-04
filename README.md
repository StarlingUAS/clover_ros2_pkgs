# Clover Core Libraries support for ROS2 

This is a ROS2 port of the libraries written by CopterExpress for use with the Clover Drone Platform. In particular the following:

- The `clover` library from their [clover project](https://github.com/CopterExpress/clover)
- The `ros_led` project [ros_led](https://github.com/CopterExpress/ros_led). 

## Packages

- **clover_ros2**: a selection of the core clover libraries ported to ROS2
- **led_msgs**: the led_msgs from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **led_msgs_test**: Python ROS2 testing for led_msgs from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **ws281x**: led drivers ros2 nodes from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2 
- **ws281x_test** test package for ws281x drivers

## Clover ROS2

This is a port of some of the core source files of the clover packages. We only port the files which are relevant to the sensor. We do not need the control node as that is handled by mavros for us.

### *clover_led* Node
This node controls higher level and more complex functionality of the led lights. It exposes the `set_effect` service which takes a clover `setEffect` message. 

```
string effect
uint8 r
uint8 g
uint8 b
float32 duration
bool notify
uint8 brightness
---
bool success
string message
```

where the `effect` parameter can be one of the following:

- `fill`: block fill colours
- `blink`: blink colour at a `blink_rate` parameter
- `blink_fast`: blink colour at a `blink_fast_rate` parameter
- `fade`: performs a fade from current colour to next colour at a `fade_period` parameter
- `wipe`: performs a wipe from current colour to next colour at a `wipe_period` parameter 
- `flash`: flashes the colour `flash_number` amount of times with `flash_period` time inbetween. Then returns to previous effect if fill, fade or wipe 
- `rainbow_fill`: fills the leds with rainbow
- `rainbow`: fills the led with rainbow

The `duration` parameter is in seconds and details the length of time an effect should last. Not specifying this parameter, or setting to 0.0 gives indefinite duration to the effect

The `brightness` parameter (0 - 255) details the overall brightness of the effect. Defaults to 70 if not specified.

The `notify` parameter triggers the request as a notification. A notification will override the current effect with the notify effect for the duration specified. Once the duration is over, the previous effect will restart. By default, all notifies last 2 seconds and are set to maximum brightness unless otherwise specified.

#### Mavros State

This node automatically attempts to subscribe to the `mavros/state` topic. When a change in state (e.g. arm/disarm or mode) is detected, a notification of a given set of styles and colours is triggered. See the list of triggers in [`led.cpp`](clover_ros2/src/led.cpp#L416)

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