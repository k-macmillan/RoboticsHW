# The bug 1 algorithm [CLH+05]
# Input A point robot with a tactile sensor
# Output A path to the qgoal or a conclusion no such path exists.
# while True do
#     repeat
#         From qLi−1 move toward qgoal
#     until qgoal is reached or obstacle is encountered at hit point qHi
#     if Goal is reached then Exit endif
#     repeat
#         Follow obstacle boundary
#     until qgoal is reached or qHi is re-encountered.
#     Determine the point qLi on the prmtr has the shrt distance to the goal
#     Go to qLi
#     if the robot were to move toward the goal then
#         Conclude qgoal is not reachable and exit
#     endif
# end while

from std_msgs.msg import Float32, ByteMultiArray
from geometry_msgs.msg import Pose2D
from RosController import *
from struct import *
import numpy as np


class Problem10a(RosController):
    def __init__(self):
        super(Problem10a, self).__init__()
        self.b_speed = 0.50
        self.hit_obj = False
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
                                                '/bug/wheel_left')
        self.pub_right = wheels.create_publisher(Float32,
                                                 '/bug/wheel_right')
        self.msg_wheels = Float32()
        self.setVel(self.b_speed, self.b_speed)

    def setVel(self, left=None, right=None):
        if left is not None:
            self.msg_wheels.data = left
            self.pub_left.publish(self.msg_wheels)
        if right is not None:
            self.msg_wheels.data = right
            self.pub_right.publish(self.msg_wheels)
        print('L: {}\nR: {}'.format(left, right))

    def setupBump(self):
        bump = self.makeNode('Bump')
        self.pub_bump = bump.create_subscription(ByteMultiArray,
                                                 '/bug/touch',
                                                 self.bumpCallback)

    def setupGPS(self):
        gps = self.makeNode('GPS')
        self.gps = gps.create_subscription(Pose2D,
                                           '/bug/GPS',
                                           self.gpsCallback)

    def bumpCallback(self, msg):
        self.hit_obj = False
        for i in range(len(msg.data)):
            hit = unpack('b', msg.data[i])[0]
            if hit:
                self.hit_obj = True
                if self.last_bump != i:
                    self.last_bump = i
                    self.bumps = 0
                if i > 9:
                    speed = np.log2(i / 9.0)
                    self.setVel(-speed + self.b_speed, speed + self.b_speed)
                    print('Bumped: ', i)
        if self.last_bump < 10:
            self.setVel(self.b_speed, self.b_speed)

        # If we hit an object turn right, otherwise move forward
        # if hit_obj and self.bumps < self.N:
        #     self.bumps += 1
        #     self.setVel(0.0, 3.0)
        # else:
        #     self.bumps = 0
        #     self.setVel(3.0, 3.0)

    def gpsCallback(self, msg):
        # print('x,y:   {}, {}'.format(msg.x, msg.y))
        # print('\nGPS theta: {}'.format(msg.theta))

        # Calc bump x,y:
        if self.hit_obj:
            theta1 = (self.last_bump * 10 - 90) * np.pi / 180.0 + msg.theta
            x_bump = np.cos(theta1 + msg.x)
            y_bump = np.sin(theta1 + msg.y)
            
            # print('Center : ({}, {})'.format(msg.x, msg.y))
            # print('Bump at: ({}, {})'.format(np.cos(theta1) + msg.x,
            #                                  np.sin(theta1) + msg.y))

        # If we are not bumping into something make progress towards goal
        # if self.bumps == 0:
        #     # we need to move to goal
        #     theta = msg.theta
        #     twopi = 2 * pi
        #     wraps = int(theta / twopi)
        #     theta = theta - (wraps * twopi)
        #     # To control slowdown as we approach 0
        #     if theta < -pi:
        #         theta = twopi + theta
        #     k = 0.25

        #     beta = arctan2(self.goal[1] - msg.y, self.goal[0] - msg.x) - pi / 2
        #     alpha = beta - theta
        #     L = 3.0 + k * alpha
        #     R = 3.0 - k * alpha
        #     self.setVel(L, R)
        pass


if __name__ == '__main__':
    p10a = Problem10a()
    p10a.run()
