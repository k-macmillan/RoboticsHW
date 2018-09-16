import numpy as np
import matplotlib.pyplot as plt


class twolink():
    """ This class is meant for fk and ik operations around a 2-link manipulator. """
        
    def __init__(self, length1, length2):
        """ Class initialization """
        self.a1 = length1
        self.a2 = length2

    def fk(self, theta1, theta2):
        """ Calculate the forward kinematics to determine the x & y values """
        x = self.a2 * np.cos(theta1 + theta2) + self.a1 * np.cos(theta1)
        y = self.a2 * np.sin(theta1 + theta2) + self.a1 * np.sin(theta1)
        return x,y

    def ik(self, x, y):
        """ Calculates the inverse kinematics to determine the theta1 & theta2 values """
        D = (x*x + y*y - self.a1 * self.a1 - self.a2 * self.a2) / (2 * self.a1 * self.a2)
        theta2 = np.arctan2(np.sqrt(1 - D * D), D)
        gamma = np.arctan2((self.a2 * np.sin(theta2)), (self.a1 + self.a2 * np.cos(theta2)))
        theta1 = np.arctan2(y, x) - gamma

        return theta1, theta2

    def plot(self, funct, show = True):
        """ Adds a function to the plot and shows it or not """
        plt.plot(funct[0], funct[1])
        if show:
            plt.show()

        



def func1():
    x = np.arange(0, 25, 0.05)
    y = 25 - x
    return x,y

def func2():
    t = np.arange(0, np.pi, 0.05)
    x = 10 * np.cos(t) + 15
    y = 10 * np.sin(t)
    return x,y


if __name__ == "__main__":
    t = twolink(15, 15)
    f1 = func1()
    f2 = func2()

    # Plot function one
    t.plot(f1)
    ikf1 = t.ik(f1[0], f1[1])
    # Plot the arm movement
    t.plot(ikf1)

    # Plot function one
    t.plot(f2)
    ikf2 = t.ik(f2[0], f2[1])
    # Plot the arm movement
    t.plot(ikf2)
    