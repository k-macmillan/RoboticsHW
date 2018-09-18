import numpy as np                              # Numerical library
from std_msgs.msg import Float32MultiArray      # Message type
from ROSwrapper.nodecontrol import NodeControl  # ROS2 controller
from Problem3_2a import line1                   # Line generator
from iknode import IkNode
from iknode2 import IkNode2


class twolink():
    """ This class is meant for fk and ik operations around a 2-link manipulator.
        This was updated from problem 10 to allow for starting theta values.
    """

    def __init__(self, length1, length2, path, rate):
        """ Class initialization """
        self.a1 = length1
        self.a2 = length2
        self.x = path[0]
        self.y = path[1]
        self.index = 0
        self.pts = zip(path[0], path[1])
        self.theta = (0.0, 0.0)

        # ROS init
        nc = NodeControl()
        nc.addnode(IkNode(name='node_xy',
                          obj=self,
                          pub_data_type=Float32MultiArray,
                          pub_chan='/physData',
                          pub_rate=5,
                          pub_data=self.pts))
        nc.addnode(IkNode(name='node_theta_magic',
                          obj=self,
                          sub_data_type=Float32MultiArray,
                          sub_chan='/physData',
                          pub_data_type=Float32MultiArray,
                          pub_chan='/thetaData',
                          pub_data=self.theta))
        nc.addnode(IkNode2(name='node_dual_sub',
                           obj=self,
                           sub_data_type=Float32MultiArray,
                           sub_chan=('/physData', '/thetaData')))

        nc.run()

    def getik(self, xy):
        """ Calculates the inverse kinematics to determine the theta1 & theta2
            values
        """
        x = xy[0]
        y = xy[1]
        theta1 = 0.0
        theta2 = 0.0
        D = (x * x + y * y - self.a1 * self.a1 - self.a2 * self.a2)\
            / (2 * self.a1 * self.a2)
        theta2 = np.arctan2(np.sqrt(1 - D * D), D)
        gamma = np.arctan2((self.a2 * np.sin(theta2)),
                           (self.a1 + self.a2 * np.cos(theta2)))
        theta1 = np.arctan2(y, x) - gamma

        return theta1, theta2

    def getfk(self, thetas):
        """ Calculate the forward kinematics to determine the x & y
            values
        """
        theta1 = thetas[0]
        theta2 = thetas[1]
        x = self.a2 * np.cos(theta1 + theta2) + \
            self.a1 * np.cos(theta1)
        y = self.a2 * np.sin(theta1 + theta2) + \
            self.a1 * np.sin(theta1)
        return x, y

    # def plot(self, funct, show=True):
    #     """ Adds a function to the plot and shows it or not """
    #     plt.plot(funct[0], funct[1])
    #     if show:
    #         plt.show()


def main():
    twolink(10, 10, path=line1(0, 10, 100), rate=5)


if __name__ == "__main__":
    main()
