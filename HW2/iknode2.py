from ROSwrapper.rosnode import RosNode             # Base class


class IkNode2(RosNode):
    """ This is an example of what a class would look like if you wanted to
        override RosNode so you can have custom publish/subscribe functions.
    """
    def __init__(self, **kwargs):
        super(IkNode2, self).__init__(**kwargs)

    def subscribe(self, topic, msg):
        if self.print_to_console:
            print('Received: {} on topic: {}'.format(msg.data, topic))
        if topic == '/physData':
            self.obj.append_plot_data_ik(msg.data)
        elif topic == '/thetaData':
            self.obj.append_plot_data_fk(self.runfk(msg.data))

    def runfk(self, thetas):
        return self.obj.getfk(thetas)
