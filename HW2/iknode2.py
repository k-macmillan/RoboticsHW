from ROSwrapper.rosnode import RosNode             # Base class
import matplotlib.pyplot as plt


class IkNode2(RosNode):
    """ This is an example of what a class would look like if you wanted to
        override RosNode so you can have custom publish/subscribe functions.
    """
    def __init__(self, **kwargs):
        super(IkNode2, self).__init__(**kwargs)

    def sub_pub(self, msg):
        ik = self.runik(msg.data)
        fk = self.runfk(ik)
        # self.pub_msg.data =
        print('Theta points: ', self.pub_msg.data)
        self.publisher.publish(self.pub_msg)

    def runik(self, xy):
        return self.obj.getik(xy)

    def runfk(self, thetas):
        return self.obj.getfk(thetas)

    def plotdata(self, xy, thetas):
        print('Plotting...')


# TODO: Allow node to have multiple subscriptions
