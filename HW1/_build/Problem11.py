import numpy as np
import matplotlib.pyplot as plt
n = 50 # for linspace

class twolink():
    """ This class is meant for fk and ik operations around a 2-link manipulator. 
        This was updated from problem 10 to allow for starting theta values. """

    def __init__(self, length1, length2, *args):
        """ Class initialization """
        self.a1 = length1
        self.a2 = length2
        self.theta1 = [0.0]
        self.theta2 = [0.0]
        self.x = [0.0]
        self.y = [0.0]

        # Assign theta1/2 if they were passed in
        if len(args) != 2:
            return

        self.theta1 = args[0]
        self.theta2 = args[1]


    def setXY(self, x, y):
        """ Set the class's x and y values """
        self.x = x
        self.y = y

    def setThetas(self, theta1, theta2):
        """ Set the class theta values """
        self.theta1 = theta1
        self.theta2 = theta2
        

    def fk(self):
        """ Calculate the forward kinematics to determine the x & y values """
        self.x = self.a2 * np.cos(self.theta1 + self.theta2) + self.a1 * np.cos(self.theta1)
        self.y = self.a2 * np.sin(self.theta1 + self.theta2) + self.a1 * np.sin(self.theta1)
        return self.x, self.y

    def ik(self):
        """ Calculates the inverse kinematics to determine the theta1 & theta2 values """
        D = (self.x*self.x + self.y*self.y - self.a1 * self.a1 - self.a2 * self.a2) / (2 * self.a1 * self.a2)
        self.theta2 = np.arctan2(np.sqrt(1 - D * D), D)
        gamma = np.arctan2((self.a2 * np.sin(self.theta2)), (self.a1 + self.a2 * np.cos(self.theta2)))
        self.theta1 = np.arctan2(self.y, self.x) - gamma

        return self.theta1, self.theta2

    def plot(self, funct, show = True):
        """ Adds a function to the plot and shows it or not """
        plt.plot(funct[0], funct[1])
        if show:
            plt.show()


# The following functions could be turned into a polygon class but was unecessary for this one problem
def line1():
    y = np.linspace(0, 15, n, endpoint=True)
    x = np.linspace(5, 5, n, endpoint=True)
    return x, y

def line2():
    y = np.linspace(15, 15, n, endpoint=True)
    x = np.linspace(5, 20, n, endpoint=True)
    return x, y

def line3():
    y = np.linspace(15, 0, n, endpoint=True)
    x = np.linspace(20, 20, n, endpoint=True)
    return x, y

def line4():
    y = np.linspace(0, 0, n, endpoint=True)
    x = np.linspace(20, 5, n, endpoint=True)
    return x, y


if __name__ == "__main__":
    t = twolink(15, 15)
    l1 = line1()
    l2 = line2()
    l3 = line3()
    l4 = line4()

    # Add lines to the plot so you can see each individually
    t.plot(l1, False)
    t.plot(l2, False)
    t.plot(l3, False)
    t.plot(l4)

    # Place all points into two np arrays for kinematics and plotting
    x = np.append(l1[0], [l2[0], l3[0], l4[0]])
    y = np.append(l1[1], [l2[1], l3[1], l4[1]])
    t.setXY(x,y)
    t.plot(t.ik())
