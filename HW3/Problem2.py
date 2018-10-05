from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from RosController import *


class Problem2(RosController):
    def __init__(self):
        super(Problem2, self).__init__()
        self.setupWheels()
        self.setupGPS()

    def setupWheels(self):
        wheels = self.makeNode('Wheels')
        self.pub_left = wheels.create_publisher(Float32,
                                                '/robot0/wheel_left')
        self.pub_right = wheels.create_publisher(Float32,
                                                 '/robot0/wheel_right')
        self.msg_wheels = Float32()
        self.setVel(2.0, 2.0)

    def setVel(self, left=None, right=None):
        if left is not None:
            self.msg_wheels.data = left
            self.pub_left.publish(self.msg_wheels)
        if right is not None:
            self.msg_wheels.data = right
            self.pub_right.publish(self.msg_wheels)

    def setupGPS(self):
        gps = self.makeNode('GPS')
        gps.create_subscription(Pose2D,
                                '/robot0/GPS',
                                self.gpsPublish)

    def gpsPublish(self, msg):
        print('x,y:   {}, {}'.format(msg.x, msg.y))
        print('theta: {}'.format(msg.theta))


if __name__ == '__main__':
    p2 = Problem2()
    p2.run()
