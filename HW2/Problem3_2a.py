#! /usr/bin/python3

import numpy as np
from std_msgs.msg import Float32MultiArray
import rclpy

rclpy.init()

msg = Float32MultiArray()

node_xy = rclpy.create_node('node_xy')
pub = node_xy.create_publisher(Float32MultiArray, '/physData')

def line1(a, b, pts):
    """ Generates the line y = 15 - x for 0≤x≤10 """
    x = np.arange(a, b, np.fabs((b - a) / pts))
    y = 15 - x
    return x, y

pts = line1(0, 10, 100)
pts = zip(pts[0], pts[1])

def __publish():
    try:
        msg.data = next(pts)
        print(msg.data)
        pub.publish(msg)
    except StopIteration:
        pass


def main():
    """ Main function, runs program """
        
    timer = node_xy.create_timer(0.2, __publish)
    rclpy.spin(node_xy)
    node_xy.destroy_timer(timer)
    node_xy.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
