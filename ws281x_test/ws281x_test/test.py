import rclpy
from rclpy.node import Node

import time

from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState

def gen_led_state(i, r, g, b):
    s = LEDState()
    s.index = i
    s.r = r
    s.g = g
    s.b = b
    return s

class WS281XTest(Node):

    def __init__(self):
        super().__init__('ws281x_test')

        self.led_count = self.get_parameter_or('led_count', 30)
        self.set_leds = self.create_client(SetLEDs, '/set_leds')

    def fill_strip(self, red, green, blue):
        req = SetLEDs.Request()
        req.leds = [gen_led_state(i, red, green, blue) for i in range(self.led_count)]
        return self.set_leds.call_async(req) # returns a future object

def main(args=None):
    rclpy.init(args=args)

    wtest = WS281XTest()

    fut = wtest.fill_strip(200, 200, 200)
    rclpy.spin_until_future_complete(wtest, fut)
    time.sleep(1)
    

    fut = wtest.fill_strip(0, 200, 0)
    rclpy.spin_until_future_complete(wtest, fut)
    time.sleep(1)

    fut = wtest.fill_strip(0, 0, 200)
    rclpy.spin_until_future_complete(wtest, fut)
    time.sleep(1)

    fut = wtest.fill_strip(200, 0, 0)
    rclpy.spin_until_future_complete(wtest, fut)
    time.sleep(1)

    fut = wtest.fill_strip(0, 0, 0)
    rclpy.spin_until_future_complete(wtest, fut)

    wtest.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


