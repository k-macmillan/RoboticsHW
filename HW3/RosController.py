import rclpy
from rclpy.executors import SingleThreadedExecutor


class RosController(object):
    """ Controller for nodes """
    def __init__(self):
        try:
            rclpy.init()
            print('rclpy initialized...')
        except BaseException:
            # rclpy is already initialized
            pass
        self.executor = SingleThreadedExecutor()
        self.nodes = []

    def makeNode(self, name, verbose=True):
        """ Makes a node and then prints to the screen that the node was
            created
        """
        node = rclpy.create_node(name)
        self.nodes.append(node)
        if verbose:
            print('{} node created...'.format(name))
        return node

    def addTimer(self, node, time, callback, verbose=True):
        """ Creates a timer for the node. Prints creation to screen """
        node.create_timer(time, callback)
        if verbose:
            print('Timer created for {}...'.format(node.get_name()))

    def shutdownOverride(self):
        """ This method exists so that a derived class can shutdown properly"""
        pass

    def __shutdown(self):
        """ Walks each node, removing timers and destroying the node """
        print('\nCleaning up...')
        self.shutdownOverride()
        for node in self.nodes:
            name = node.get_name()

            # Clean up timers
            while (len(node.timers) > 0):
                timer = node.timers.pop()
                node.destroy_timer(timer)
                print('Timer destroyed for {} node...'.format(name))

            node.destroy_node()
            print('Destroyed {} node...'.format(name))

        self.executor.shutdown()
        print('executor shut down...')
        rclpy.shutdown()
        print('rclpy shut down...')

    def run(self):
        """ Runs all the nodes that have been added to the controller """
        for node in self.nodes:
            self.executor.add_node(node)

        try:
            self.executor.spin()
        except KeyboardInterrupt:
            self.__shutdown()
            exit(1)
