import numpy as np
from ROSwrapper.nodecontrol import NodeControl
from ROSwrapper.rosnode import RosNode
from wheelnode import WheelNode
from std_msgs.msg import Float32MultiArray, Int8


class DiffDrive():
    def __init__(self, start, end, hz):
        self.pub_rate = hz
        self.step_size = 1.0 / hz
        t = np.arange(start, end + self.step_size, self.step_size)  # inclusive with end
        self.phi_1 = 2.0 + 2.0 * np.exp(-t)
        self.phi_2 = 2.0 + np.exp(-2.0 * t)
        self.phi_data = zip(self.phi_1, self.phi_2)
        self.inactive = False

        self.initRosNodes()

    def initRosNodes(self):
        self.nc = NodeControl()
        self.nc.addnode(WheelNode(name='Control',
                                  obj=self,
                                  pub_chan='/WheelVel',
                                  pub_rate=self.pub_rate,
                                  pub_data_type=Float32MultiArray,
                                  pub_data=self.phi_data,
                                  print_to_console=True))
        self.active_node = self.nc.addnode(RosNode(name='Active',
                                                   obj=self,
                                                   pub_chan='/Active',
                                                   pub_rate=1,
                                                   pub_data_type=Int8,
                                                   pub_data=1,
                                                   print_to_console=True))
        self.nc.run()

    def done(self):
        self.active_node.pub_msg.data = 0
        self.inactive = True


if __name__ == '__main__':
    dd = DiffDrive(start=0, end=10, hz=10)
