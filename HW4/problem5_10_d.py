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

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from RosController import *
from struct import *
import numpy as np
from enum import Enum


class State(Enum):
    MOV_TO_GOAL_1 = 1
    CIRCLE_OBST = 2
    ROUTE_BEST = 3
    MOV_TO_GOAL_2 = 4


class Problem10d(RosController):
    def __init__(self):
        super(Problem10d, self).__init__()
        self.b_speed = 0.50
        self.hit_obj = False
        self.obstacle = []
        self.last_xy = None
        self.goal = (0.0, 20.0)
        self.last_bump = -1
        self.temp_goal = (0.0, 0.0)
        self.obj_break = (-1, -1)
        self.state = State.MOV_TO_GOAL_1
        self.origin = None
        self.setupWheels()
        self.setupBump()
        self.setupGPS()

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
        # print('L: {}\nR: {}'.format(left, right))

    def setupBump(self):
        lidar = self.makeNode('Lidar')
        self.pub_lidar = lidar.create_subscription(LaserScan,
                                                   '/bug/LIDAR',
                                                   self.lidarCallback)

    def setupGPS(self):
        gps = self.makeNode('GPS')
        self.gps = gps.create_subscription(Pose2D,
                                           '/bug/GPS',
                                           self.gpsCallback)

    def lidarCallback(self, msg):
        self.hit_obj = False
        min_dist = float("inf")
        min_idx = 0
        for i in range(len(msg.ranges)):            
            if msg.ranges[i] < min_dist:
                self.hit_obj = True
                min_dist = msg.ranges[i]
                min_idx = i

        if self.hit_obj:
            self.last_bump = min_idx        
            if self.last_bump > 11:
                speed = np.log2(i / 8.0)
                self.setVel(-speed + self.b_speed, speed + self.b_speed)
            if self.last_bump < 11:
                    # print('Set constant speed...')
                    speed = np.log2(self.last_bump / 8.0)
                    self.setVel(speed + self.b_speed, -speed + self.b_speed)
        # print('Min idx: {}\nMin dist: {}'.format(min_idx, min_dist))
        
        # if self.state != State.MOV_TO_GOAL_2:
        #     for i in range(len(msg.data)):
        #         hit = unpack('b', msg.data[i])[0]
        #         if hit:
        #             self.hit_obj = True
        #             if self.last_bump != i:
        #                 self.last_bump = i
        #             if i > 11:
        #                 speed = np.log2(i / 8.0)
        #                 self.setVel(-speed + self.b_speed, speed + self.b_speed)
        #                 # print('Bumped: ', i)
        #     if self.last_bump < 11:
        #         # print('Set constant speed...')
        #         speed = np.log2(self.last_bump / 8.0)
        #         self.setVel(speed + self.b_speed, -speed + self.b_speed)

    def distToGoal(self, msg):
        return np.sqrt((self.goal[0] - msg.x)**2 + (self.goal[1] - msg.y)**2)

    def gpsCallback(self, msg):
        print('x,y:   {}, {}'.format(msg.x, msg.y))
        print('\nGPS theta: {}'.format(msg.theta))
        # self.setVel(self.b_speed, self.b_speed)

        # Calc bump x,y:
        if self.hit_obj and msg.theta != float('nan'):
            theta1 = (self.last_bump * 10 - 90) * np.pi / 180.0 + msg.theta
            x_bump = np.cos(theta1) + msg.x
            y_bump = np.sin(theta1) + msg.y
            print('Theta: {}\nX: {}\nY: {}'.format(theta1, x_bump, y_bump))
            xy = (int(x_bump), int(y_bump))  # Discretize
            if self.last_bump > 9:
                self.temp_goal = theta1

            if xy not in self.obstacle:
                self.obstacle.append(xy)
                if len(self.obstacle) > 3:
                    theta = msg.theta
                    twopi = 2 * np.pi
                    wraps = int(theta / twopi)
                    theta = theta - (wraps * twopi)
                    # To control slowdown as we approach 0
                    if theta < -np.pi:
                        theta = twopi + theta

                    beta = np.arctan2(self.goal[1] - msg.y,
                                      self.goal[0] - msg.x) - np.pi / 2.0
                    alpha = beta - theta

                    if -0.5 < alpha < 0.5:
                        print('Rotating...')
                        # self.state = State.ROUTE_BEST
                        self.setVel(-self.b_speed * alpha,
                                    self.b_speed * alpha)
                        if -0.125 < alpha < 0.125:
                            self.state = State.MOV_TO_GOAL_2

        elif self.state == State.MOV_TO_GOAL_2:
            theta = msg.theta
            twopi = 2 * np.pi
            wraps = int(theta / twopi)
            theta = theta - (wraps * twopi)
            # To control slowdown as we approach 0
            if theta < -np.pi:
                theta = twopi + theta

            beta = np.arctan2(self.goal[1] - msg.y,
                              self.goal[0] - msg.x) - np.pi / 2.0
            alpha = beta - theta

            if -0.5 < alpha < 0.5:
                print('Moving to goal now...\n')
                dist = self.distToGoal(msg)
                if dist > 0.50:
                    speed = self.b_speed
                else:
                    speed = 0.0
                self.setVel(speed, speed)


if __name__ == '__main__':
    p10d = Problem10d()
    p10d.run()
