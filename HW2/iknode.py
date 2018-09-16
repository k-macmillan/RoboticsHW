from ROSwrapper.rosnode import RosNode             # Base class


class IkNode(RosNode):
    """ This is an example of what a class would look like if you wanted to
        override RosNode so you can have custom publish/subscribe functions.
    """
    def __init__(self, **kwargs):
        super(IkNode, self).__init__(**kwargs)

    def sub_pub(self, msg):
        self.pub_msg.data = self.runik(msg.data)
        print('Theta points: ', self.pub_msg.data)
        self.publisher.publish(self.pub_msg)

    def runik(self, xy):
        return self.obj.getik(xy)
