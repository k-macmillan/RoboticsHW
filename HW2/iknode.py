from ROSwrapper.rosnode import RosNode             # Base class


class IkNode(RosNode):
    """ This is an example of what a class would look like if you wanted to
        override RosNode so you can have custom publish/subscribe functions.
    """
    def __init__(self, **kwargs):
        super(IkNode, self).__init__(**kwargs)

    def sub_pub(self, topic, msg):
        self.pub_msg.data = self.runik(msg.data)
        if self.print_to_console:
            print('Received: {}'.format(msg.data))
            print('Publishing: {}'.format(self.pub_msg.data))
        self.publisher.publish(self.pub_msg)

    def runik(self, xy):
        return self.obj.getik(xy)
