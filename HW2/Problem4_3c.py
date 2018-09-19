import numpy as np
from ROSwrapper.nodecontrol import NodeControl
from ROSwrapper.rosnode import RosNode
from wheelnode import WheelNode
from fknode import FkNode
from std_msgs.msg import Float32MultiArray, Int8
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


class DiffDrive():
    def __init__(self, start, end, hz, D, L):
        self.radius = D / 2.0
        self.L = L
        self.pub_rate = hz
        self.time_step = 1.0 / hz
        t = np.arange(start, end + self.time_step, self.time_step)
        self.phi_1 = 2.0 + 2.0 * np.exp(-t)
        self.phi_2 = 2.0 + np.exp(-2.0 * t)
        self.phi_data = zip(self.phi_1, self.phi_2)
        self.inactive = False
        self.fk_data = Twist()
        self.fk_data.linear.x = 0.0
        self.fk_data.linear.y = 0.0
        self.fk_data.angular.x = 0.0
        self.plot_data_x = [0.0, ]
        self.plot_data_y = [0.0, ]
        self.showing_plot = False
        self.s_plot = plt.figure()

        self.initRosNodes()

    def initRosNodes(self):
        self.nc = NodeControl()
        self.nc.addnode(WheelNode(name='Control',
                                  obj=self,
                                  pub_chan='/WheelVel',
                                  pub_rate=self.pub_rate,
                                  pub_data_type=Float32MultiArray,
                                  pub_data=self.phi_data))

        # Naming this one to access it in done()
        self.active_node = self.nc.addnode(RosNode(name='Active',
                                                   obj=self,
                                                   pub_chan='/Active',
                                                   pub_rate=1,
                                                   pub_data_type=Int8,
                                                   pub_data=1))

        self.nc.addnode(FkNode(name='ForwardK',
                               obj=self,
                               sub_chan='/WheelVel',
                               sub_data_type=Float32MultiArray,
                               pub_chan='/RobotVel',
                               pub_data_type=Twist,
                               pub_data=self.fk_data))

        self.nc.addnode(FkNode(name='RobotPlot',
                               obj=self,
                               sub_chan='/RobotVel',
                               sub_data_type=Twist))

        self.nc.run()

    def done(self):
        self.active_node.pub_msg.data = 0
        self.inactive = True

    def fillFkData(self, w):
        thetaVel = (self.radius / (2.0 * self.L)) * (w[0] - w[1])
        theta = thetaVel * self.time_step
        gamma = (self.radius / 2.0) * (w[0] + w[1])

        # Fill Twist
        self.fk_data.linear.x = gamma * np.cos(theta)
        self.fk_data.linear.y = gamma * np.sin(theta)
        self.fk_data.angular.x = thetaVel

    def fillPlotData(self, xy):
        if len(self.plot_data_x) < 100:
            # position = x0 + v * time
            x = self.plot_data_x[-1] + xy[0] * self.time_step
            y = self.plot_data_y[-1] + xy[1] * self.time_step
            self.plot_data_x.append(x)
            self.plot_data_y.append(y)
        elif not self.showing_plot:
            self.showing_plot = True
            plt.scatter(self.plot_data_x,
                        self.plot_data_y,
                        label='Position')
            plt.xlabel('x position')
            plt.ylabel('y position')
            plt.title('Robot Path')
            plt.show()
            self.s_plot.savefig('Problem4_3c.pdf',
                                format='pdf',
                                dpi=1200)
            print('Press \"ctrl\" + \"c\" to exit')


if __name__ == '__main__':
    DiffDrive(start=0, end=10, hz=10, D=10, L=20)
