#!/usr/bin/env python

# Simulator of a LED strip.
# Note this should be run in a terminal supporting 24 bit color.

import sys
import rclpy
from rclpy.node import Node

from led_msgs.msg import LEDState, LEDStateArray
from led_msgs.srv import SetLED, SetLEDs


class LEDController(Node):
    def __init__(self):
        super().__init__('led')
        self.led_count = self.get_parameter_or('~led_count', 30)
        self.state_pub = self.create_publisher(LEDStateArray, 'state', 10)

        self.state = LEDStateArray()
        self.state.leds = [LEDState(index=index) for index in range(self.led_count)]

        self.set_led_srv = self.create_service(SetLED, 'set_led', self.set_led)
        self.set_leds_srv = self.create_service(SetLEDs, 'set_leds', self.set_leds)

    def set_led(self, req, resp):
        self.state.leds[req.index].r = int(req.r)
        self.state.leds[req.index].g = int(req.g)
        self.state.leds[req.index].b = int(req.b)
        self.print_led()
        self.state_pub.publish(self.state)
        resp.success = True
        return resp


    def set_leds(self, req, resp):
        for led in req.leds:
            self.state.leds[led.index].r = int(led.r)
            self.state.leds[led.index].g = int(led.g)
            self.state.leds[led.index].b = int(led.b)
        self.print_led()
        self.state_pub.publish(self.state)
        resp.success = True
        return resp

    def print_led(self):
        s = ''
        for led in self.state.leds:
            s += '\033[48;2;{};{};{}m '.format(led.r, led.g, led.b)
        strs = '\r{}\033[0m'.format(s)
        self.get_logger().info(strs)
        # sys.stdout.write()
        # sys.stdout.flush()


def main(args=None):

    rclpy.init(args=args)

    ledcontroller = LEDController()

    rclpy.spin(ledcontroller)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

