# Basic motion planning algorithm:
#
# Set heading towards goal
# while Not arrived at goal do
#     while No obstacle in front do
#         Move forward
#     end while
#     count = 0
#     while count <= N do
#         while Obstacle in front do
#             Turn right
#         end while
#         Move forward
#         incr count
#     end while
#     Set heading towards goal
# end while

from std_msgs.msg import Float32, ByteMultiArray
from geometry_msgs.msg import Pose2D
from RosController import *
from struct import *
from numpy import pi, arctan2


class Problem2(RosController):
    def __init__(self):
        super(Problem2, self).__init__()
        self.setupWheels()
        self.setupBump()
        self.setupGPS()
        self.goal = (0.0, 20.0)
        self.N = 3  # max bumps
        self.bumps = 0
        self.last_bump = -1

    def shutdownOverride(self):
        """ Overriden shutdown function"""
        self.setVel(0.0, 0.0)

    def setupWheels(self):
        wheels = self.makeNode('Wheels')
        self.pub_left = wheels.create_publisher(Float32,
                                                '/basic_bug/wheel_left')
        self.pub_right = wheels.create_publisher(Float32,
                                                 '/basic_bug/wheel_right')
        self.msg_wheels = Float32()
        self.setVel(3.0, 3.0)

    def setVel(self, left=None, right=None):
        if left is not None:
            self.msg_wheels.data = left
            self.pub_left.publish(self.msg_wheels)
        if right is not None:
            self.msg_wheels.data = right
            self.pub_right.publish(self.msg_wheels)

    def setupBump(self):
        bump = self.makeNode('Bump')
        self.pub_bump = bump.create_subscription(ByteMultiArray,
                                                 '/basic_bug/touch',
                                                 self.bumpCallback)

    def setupGPS(self):
        gps = self.makeNode('GPS')
        self.gps = gps.create_subscription(Pose2D,
                                           '/basic_bug/GPS',
                                           self.gpsCallback)

    def bumpCallback(self, msg):
        hit_obj = False
        for i in range(len(msg.data)):
            hit = unpack('b', msg.data[i])[0]
            if hit:
                if self.last_bump != i:
                    self.last_bump = i
                    self.bumps = 0
                hit_obj = True
                # print('Bumped: ', i)

        # If we hit an object turn right, otherwise move forward
        if hit_obj and self.bumps < self.N:
            self.bumps += 1
            self.setVel(0.0, 3.0)
        else:
            self.bumps = 0
            self.setVel(3.0, 3.0)

    def gpsCallback(self, msg):
        # print('x,y:   {}, {}'.format(msg.x, msg.y))
        # print('\nGPS theta: {}'.format(msg.theta))

        # If we are not bumping into something make progress towards goal
        if self.bumps == 0:
            # we need to move to goal
            theta = msg.theta
            twopi = 2 * pi
            wraps = int(theta / twopi)
            theta = theta - (wraps * twopi)
            # To control slowdown as we approach 0
            if theta < -pi:
                theta = twopi + theta
            k = 0.25

            beta = arctan2(self.goal[1] - msg.y, self.goal[0] - msg.x) - pi / 2
            alpha = beta - theta
            L = 3.0 + k * alpha
            R = 3.0 - k * alpha
            self.setVel(L, R)


if __name__ == '__main__':
    p2 = Problem2()
    p2.run()
