from ROSwrapper.rosnode import RosNode             # Base class


class WheelNode(RosNode):
    """ Overriden RosNode for custom publish function """
    def __init__(self, **kwargs):
        super(WheelNode, self).__init__(**kwargs)

    def publish(self):
        if not self.obj.inactive:
            try:
                self.pub_msg.data = next(self.pub_data)
            except StopIteration:
                self.obj.done()
                return False
            self.publisher.publish(self.pub_msg)
            return True
