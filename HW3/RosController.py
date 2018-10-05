import rclpy
from rclpy.executors import SingleThreadedExecutor


class RosController(object):
    def __init__(self):
        try:
            rclpy.init()
            print('rclpy initialized...')
        except BaseException:
            pass
        self.executor = SingleThreadedExecutor()
        self.nodes = []

    def makeNode(self, name):
        node = rclpy.create_node(name)
        self.nodes.append(node)
        print('{} node created...'.format(name))
        return node

    def addTimer(self, node, time, callback):
        node.create_timer(time, callback)
        print('Timer created for {}...'.format(node.get_name()))

    def shutdownOverride(self):
        """ This method exists so that a derived class can shutdown properly"""
        pass

    def __shutdown(self):
        print('\nCleaning up...')
        self.shutdownOverride()
        try:
            for node in self.nodes:
                name = node.get_name()
                while (len(node.timers) > 0):
                    timer = node.timers.pop()
                    node.destroy_timer(timer)
                    print('Timer destroyed for {} node...'.format(name))
                node.destroy_node()
                print('Destroyed {}...'.format(name))

            self.executor.shutdown()
            rclpy.shutdown()
            print('Shut down...')
        except BaseException:
            # Already shutdown
            pass

    def run(self):
        for node in self.nodes:
            self.executor.add_node(node)

        try:
            self.executor.spin()
        except KeyboardInterrupt:
            self.__shutdown()
            exit(1)
