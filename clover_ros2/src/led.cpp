#include <chrono>
#include <memory>

#include "rclpp/rclpp.hpp"
#include "clover_ros2/srv/SetLEDEffect.srv"
#include "led_msgs/msg/SetLED.h"
#include "led_msgs/msg/LEDState.h"
#include "led_msgs/msg/LEDStateArray.h"

#include "sensor_msgs/msg/BatteryState.h"
#include "mavros_msgs/msg/State.h"
#include "rosgraph_msgs/msg/Log.h"