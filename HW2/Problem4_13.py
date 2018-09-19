import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np


class DiffDrive():
    def __init__(self):
        self.pt0 = (0.0, 0.0)
        self.pt1 = (0.0, 15.0)
        self.pt2 = (5.0, 20.0)

        # Radians
        self.theta0 = 0.0
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.dist_threshold = 1.0
        self.rosinit()

    def rosinit(self):
        node_name = 'circle'
        rclpy.init()
        node = Node(node_name)

        print('Spinning: {}'.format(node_name))
        self.publeft = node.create_publisher(Float32, '/wheel_left')
        self.pubright = node.create_publisher(Float32, '/wheel_right')
        pos = node.create_subscription(Pose2D, '/GPS', self.callback)

        msg = Float32()
        msg.data = 2.0
        self.publeft.publish(msg)
        self.pubright.publish(msg)

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print('Shutting down...')
            msg.data = 0.0
            self.publeft.publish(msg)
            self.pubright.publish(msg)
            node.destroy_node()
            rclpy.shutdown()

    def dist(self, pt, xy):
        return np.sqrt((pt[0] - xy[0] * pt[0] - xy[0]) + (pt[1] - xy[1] * pt[1] - xy[1]))

    def thetadiff(self, goal, theta):
        return np.absolute(theta - goal)

    def callback(self, msg):
        print('x,y:   {},{}'.format(msg.x, msg.y))
        print('theta: {}'.format(msg.theta))
        dist = self.dist(self.pt1, (msg.x, msg.y))
        print('dist: {}'.format(dist))
        thetadist = self.thetadiff(self.theta1, msg.theta)
        print('thetadist: {}'.format(thetadist))

        # Naive approach
        if dist < self.dist_threshold and thetadist < self.dist_threshold:
            # Rotate
            print('rotate')
            new_msg = Float32()
            new_msg.data = 0.0
            self.publeft.publish(new_msg)
            new_msg.data = -0.0
            self.pubright.publish(new_msg)
        else:
            # Have obtained correct angle, now move forward
            new_msg = Float32()
            new_msg.data = 2.0
            self.publeft.publish(new_msg)
            self.pubright.publish(new_msg)



if __name__ == '__main__':
    DiffDrive()


    



