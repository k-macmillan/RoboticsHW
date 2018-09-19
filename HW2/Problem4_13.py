import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np
import itertools


class DiffDrive():
    def __init__(self, problem):
        self.problem = problem

        self.tri_pts = itertools.cycle(iter([(0.0, 0.0),
                                        (15.0, 0.0),
                                        (5.0, 20.0)]))

        self.sqr_pts = itertools.cycle(iter([(0,0),
                                        (10,0),
                                        (10,10),
                                        (0,10)]))
        self.position = (0.0, 0.0)
        self.theta = 0.0
        self.r = 0.25
        
        self.radius = 1.0
        self.L = 0.6
        self.time_step = 0.1

        # Buffer
        self.nose = (self.position[0] + self.r * np.cos(self.theta),
                     self.position[1] + self.r * np.sin(self.theta))

        # Radians
        self.tri_thetas = itertools.cycle(iter([0.0,
                                                0.000001,
                                                0.0]))
        self.sqr_thetas = itertools.cycle(iter([0.0,          # 0
                                                1.5707963,    # 90
                                                3.1415296,    # 180
                                                4.7123889]))    # 270

        self.rosinit()

    def rosinit(self):
        node_name = 'circle'
        rclpy.init()
        node = Node(node_name)
        msg = Float32()

        print('Spinning: {}'.format(node_name))
        self.publeft = node.create_publisher(Float32, '/wheel_left')
        self.pubright = node.create_publisher(Float32, '/wheel_right')
        if self.problem == 'triangle':
            self.goal_pt = next(self.tri_pts)
            self.goal_pt = next(self.tri_pts)
            self.goal_theta = next(self.tri_thetas)
            self.goal_theta = next(self.tri_thetas)
            msg.data = 1.0
            self.publeft.publish(msg)
            msg.data = -msg.data
            self.pubright.publish(msg)
            node.create_subscription(Pose2D, '/GPS', self.tri_callback)
        elif self.problem == 'square':
            self.goal_pt = next(self.sqr_pts)
            self.goal_pt = next(self.sqr_pts)
            self.goal_theta = next(self.sqr_thetas)
            self.goal_theta = next(self.sqr_thetas)
            msg.data = 1.0
            self.publeft.publish(msg)
            msg.data = -msg.data
            self.pubright.publish(msg)
            node.create_subscription(Pose2D, '/GPS', self.sqr_callback)
        else:
            # Circle            
            msg.data = 5.0
            self.publeft.publish(msg)
            msg.data = msg.data * (15.0 + 1.2) / 15
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
        return np.sqrt((pt[0] - xy[0]) * (pt[0] - xy[0]) + ((pt[1] - xy[1]) * (pt[1] - xy[1])))

    def thetadiff(self, goal, theta):
        return theta - goal


    def tri_callback(self, msg):
        print('x,y:   {}, {}'.format(msg.x, msg.y))
        print('theta: {}'.format(msg.theta))
        dist = self.dist(self.goal_pt, (msg.x, msg.y))
        print('dist: {}'.format(dist))
        thetadist = self.thetadiff(self.goal_theta, msg.theta)
        print('thetadist: {}'.format(thetadist))

        # Ordering one wiggly bot!
        if dist > 1:
            new_msg = Float32()
            new_msg.data = 2.0 + 5 * thetadist
            self.publeft.publish(new_msg)
            new_msg.data = 2.0 - 5 * thetadist
            self.pubright.publish(new_msg)
        else:
            self.goal = next(self.tri_pts)
            self.goal_theta = next(self.tri_thetas)

    def sqr_callback(self, msg):
        print('x,y:   {}, {}'.format(msg.x, msg.y))
        print('theta: {}'.format(msg.theta))
        dist = self.dist(self.goal_pt, (msg.x, msg.y))
        print('dist: {}'.format(dist))
        thetadist = self.thetadiff(self.goal_theta, msg.theta)
        print('thetadist: {}'.format(thetadist))

        # Ordering one wiggly bot!
        if dist > 1:
            new_msg = Float32()
            new_msg.data = 2.0 + 5 * thetadist
            self.publeft.publish(new_msg)
            new_msg.data = 2.0 - 5 * thetadist
            self.pubright.publish(new_msg)
        else:
            self.goal_pt = next(self.sqr_pts)
            self.goal_theta = next(self.sqr_thetas)

if __name__ == '__main__':
    DiffDrive(problem='square')


    



