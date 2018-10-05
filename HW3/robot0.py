import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from rclpy.executors import SingleThreadedExecutor


class RosControl():
    def __init__(self):
        try:
            rclpy.init()
            print('rclpy initialized...')
        except BaseException:
            pass
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        self.timers = []

        self.setupWheels()
        self.setupGPS()

        for node in self.nodes:
            self.executor.add_node(node)

        try:
            self.executor.spin()
        except KeyboardInterrupt:
            self.shutdown()
            exit(1)

    def setupWheels(self):
        self.wheels = Node('Wheels')
        self.publeft = self.wheels.create_publisher(Float32,
                                                    '/robot0/wheel_left')
        self.pubright = self.wheels.create_publisher(Float32,
                                                     '/robot0/wheel_right')
        self.msg_wheels = Float32()
        self.setVel(2.0, 2.0)
        self.nodes.append(self.wheels)
        print('Wheels node created...')

    def setupGPS(self):
        self.gps = Node('GPS')
        self.gps_sub = self.gps.create_subscription(Pose2D,
                                                    '/robot0/GPS',
                                                    self.gpsPublish)

        self.nodes.append(self.gps)
        print('GPS node created...')

    def gpsPublish(self, msg):
        print('x,y:   {}, {}'.format(msg.x, msg.y))
        print('theta: {}'.format(msg.theta))

    def setVel(self, left=None, right=None):
        if left is not None:
            self.msg_wheels.data = left
            self.publeft.publish(self.msg_wheels)
        if right is not None:
            self.msg_wheels.data = right
            self.pubright.publish(self.msg_wheels)

    def shutdown(self):
        print('\nCleaning up...')
        try:
            self.setVel(0.0, 0.0)
            for node in self.nodes:
                name = node.get_name()
                node.destroy_node()
                print('Destroyed {}...'.format(name))

            self.executor.shutdown()
            rclpy.shutdown()
            print('Shut down...')
        except BaseException:
            print('Got an error!')
            # Already shutdown
            pass


if __name__ == '__main__':
    rc = RosControl()
