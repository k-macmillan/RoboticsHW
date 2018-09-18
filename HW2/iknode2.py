from ROSwrapper.rosnode import RosNode             # Base class
# import matplotlib.pyplot as plt


class IkNode2(RosNode):
    """ This is an example of what a class would look like if you wanted to
        override RosNode so you can have custom publish/subscribe functions.
    """
    def __init__(self, **kwargs):
        super(IkNode2, self).__init__(**kwargs)

    def subscribe(self, topic, msg):
        print('Received: {} on topic: {}'.format(msg.data, topic))
        # print()
        # print((msg.data.__str__.__doc__))

    def runik(self, xy):
        return self.obj.getik(xy)

    def runfk(self, thetas):
        return self.obj.getfk(thetas)

# Write a program that subscribes to both /physData and /thetaData.
# The program should plug the angles into the forward kinematics and
# check against the data in /physData.
# It should plot the original curve in green and the “check” in blue.
