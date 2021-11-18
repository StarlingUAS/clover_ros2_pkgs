# Clover Core Libraries support for ROS2

This is a ROS2 port of the libraries written by CopterExpress for use with the Clover Drone Platform. In particular the following:

- The `clover` library from their [clover project](https://github.com/CopterExpress/clover)
- The `ros_led` project [ros_led](https://github.com/CopterExpress/ros_led).
- The `vl53l1x` project [vl53l1x_ros2](https://github.com/mhl787156/vl53l1x_ros2)

## Installation

### Local Development

Download the repository into your ros2 development workspace:
```
cd ~/dev_ws/src
git clone --recurse-submodules https://github.com/UoBFlightLab/clover_ros2_pkgs
```

Then build using colcon
```bash
cd ~/dev_ws/
colcon build
# Or specific packages using `colcon build --packages-select clover_ros2`
```

### Docker

This project is also available as the `starling-clover` docker container for both amd64 and arm64 as part of [**Project Starling**](https://github.com/UoBFlightLab/ProjectStarling).

```
docker pull uobflightlabstarling/starling-clover:latest
docker run -it --rm --name clover --network=host uobflightlabstarling/starling-clover
```

This container also include mavros and will run `clover.launch.xml`. It is recommended that you run with `network=host` to enable ROS2 communication.

See [**Project Starling**](https://github.com/UoBFlightLab/ProjectStarling) for more details.

## Running

Make sure the local setup is sourced: `source install/setup.bash`

Everything can be run wiht the following:

```bash
ros2 launch clover_ros2 clover.launch.xml
```

## Packages

- **clover_ros2**: a selection of the core clover libraries ported to ROS2
- **led_msgs**: the led_msgs from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **led_msgs_test**: Python ROS2 testing for led_msgs from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **ws281x**: led drivers ros2 nodes from [ros_led](https://github.com/CopterExpress/ros_led) ported to ROS2
- **ws281x_test** test package for ws281x drivers
- **vl53l1x_ros2** rangefinder driver ros2 nodes from [vl53l1x_ros2](https://github.com/mhl787156/vl53l1x_ros2). Note that this is a git submodule. See the README in the module itself for further details.
- **simple_offboard**: A basic [offboard controller](https://github.com/StarlingUAS/starling_simple_offboard) which is a ROS2 port of the clover simple_offboard controller. 

## Clover ROS2

This is a port of some of the core source files of the clover packages. We only port the files which are relevant to the sensor. We do not need the control node as that is handled by mavros for us.

### *simple_offboard* Node
This node is intended to simplify the programming of autonomous drone flight (`OFFBOARD` flight mode). It allows the setting of desired flight tasks and automatically transforms coordinates between frames. It is a high level system for interacting with the flight controller.

This is a direct port of the `simple_offboard` module from the clover project. See the following documentation from [Coex](https://clover.coex.tech/en/simple_offboard.html).

It also provides a novel trajectory following node that is not included in the original clover project.

See [simple_offboard docs](starling_simple_offboard) for further details

### *clover_led* Node
This node controls higher level and more complex functionality of the led lights. It exposes the `set_effect` service which takes a clover `setLEDEffect` request message.

```
string effect
uint8 r
uint8 g
uint8 b
uint8 brightness
float32 duration
uint8 priority
bool base 
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
- `reset`: A special parameter which resets the base effect to the default. 

The `brightness` parameter (0 - 255) details the overall brightness of the effect. Defaults to 255 if not specified.

The `priority` is an integer, by default, from 0 to 9 (set by parameter `num_priority_levels`). The controller implements a unique-priority priority queue like structure. That looks a bit like the following (truncated to 5 levels rather than the actual 9):

| Priority: | 0                 | 1 | 2 | 3 | 4              | 5 |
|-----------|-------------------|---|---|---|----------------|---|
| Effect:   | fill(100,100,100) |   |   |   | blink(255,0,0) |   |

For each priority level, there is only one saved effect. Sending a second effect of the same priority level as previous will overwrite the previous effect. Each loop the controller will select and play the highest priority 
effect. If an effect has exceeded its duration it will be removed and the next highest selected. 

The `base` parameter defines an effect as the base effect to be played if the queue is empty. This is useful for conveying a healthy connection or something similar. By default it is set as the `rainbow` effect. It can be reset by sending a `SetLEDEffect` request with `effect=reset`. Note if `base` is received the entire queue will be cleared. 

The `duration` parameter is in seconds and details the length of time an effect should last. Not specifying this parameter, or setting to 0.0 gives indefinite duration to the effect.
e 
### *mavros_led* node

This node is intended to be the drone watchdog node, monitoring the connection with mavros as well as other connections and conveying these states to the user through the LEDs. 

It automatically attempts to subscribe to the `mavros/state` topic and sends LEDEffect requests based on the current state of the connection. The specific effects are given in the parameter file [notify_params.yaml](clover_ros2/config/notify_params.yaml). The following are the key effects:

* `no_mavros`: The watchdog continuously monitors the connectiong with mavros. If there has been no message from `mavros/state` within a timeout period, the `no_mavros` effect is played. By default this is a blue quick blink with a short duration with priority 9. 
*  `base`: If a connection is found, the `base` effect is set as the base LED effect. By default this is set in the code to be a blue fill. 
* `emergency_stop_topic`: If the emergency stop is triggered on `/emergency_stop`, this effect is triggered. By default is a red quick blink with a short duration with priority 9.
* `armed`: Plays when the vehicle changes from disarmed to arm for 5 seconds
* `disarmed`: Plays when vehicle changes from armed to disarm for 5 seconds
* `manual`
* `posctl`: Plays on position control mode
* `offboard`: Plays on offboard control mode 

## LED Control

### Custom LED ROS messages (led_msgs package)

This is a port of the led_msgs ros1 package written by CopterExpress. See the original [files here](https://github.com/CopterExpress/ros_led/tree/master/led_msgs).

This is organised as an `ament_cpp` project so the python test file had to be moved to a separate ROS package.

> Note: custom msg/srv/actions cannot currently be included in `ament_python` build project (13/05/2021)

### WS281x LED Strip Support for ROS2 (ws281x package)

This is a port of the WS281x ROS1 package written by CopterExpress. See the original [ros1 here](https://github.com/CopterExpress/ros_led/tree/master/ws281x)

This is the LED strip used on the Clover Drone platform

This package contains the WS281x LED strip driver for ROS (on a rapsberry pi). Based on the [rpi_ws281x library](https://github.com/jgarff/rpi_ws281x)

## Rangefinder

### vl53l1x Rangefinder support for ROS2

This is STM [VL53L1X](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) time-of-flight rangefinder driver for ROS2. Tested on a Raspberry Pi 3 and 4 with [CJMCU-531](https://ru.aliexpress.com/item/VL53L1X/32911692450.html) board.

See the [project here](https://github.com/mhl787156/vl53l1x_ros2) for more details. This is a ROS2 port of the [original project here](https://github.com/okalachev/vl53l1x_ros).

## License

As per the original Clover source code, this ROS2 port is also covered under the MIT License.