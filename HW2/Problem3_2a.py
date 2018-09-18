#! /usr/bin/python3

import numpy as np
from ROSwrapper.nodecontrol import NodeControl
from ROSwrapper.rosnode import RosNode
from std_msgs.msg import Float32MultiArray


def line1(a, b, pts):
    """ Generates the line y = 15 - x for 0≤x≤10 """
    x = np.arange(a, b, np.fabs((b - a) / pts))
    y = 15 - x
    return x, y


def main():
    """ Main function, runs program """
    pts = line1(0, 10, 100)
    pts = zip(pts[0], pts[1])

    nc = NodeControl()
    nc.addnode(RosNode(name='pub_node',
                       pub_data_type=Float32MultiArray,
                       pub_chan='/physData',
                       pub_rate=5,
                       pub_data=pts,
                       print_to_console=True))
    nc.run()


if __name__ == "__main__":
    main()
