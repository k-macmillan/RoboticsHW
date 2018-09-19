from ROSwrapper.rosnode import RosNode             # Base class


class FkNode(RosNode):
    """ Overriden RosNode for custom publish function """
    def __init__(self, **kwargs):
        super(FkNode, self).__init__(**kwargs)

    def sub_pub(self, topic, msg):
        """ Used by the ForwardK node to hear subscription and to publish """
        if not self.obj.inactive:
            self.obj.fillFkData(msg.data)
            self.pub_msg = self.obj.fk_data
            self.publisher.publish(self.pub_msg)
            return True
        else:
            return False

    def subscribe(self, topic, msg):
        """ Used by RobotPlot node to perform plot operations """
        if topic == '/RobotVel':
            self.obj.fillPlotData((msg.linear.x, msg.linear.y))
