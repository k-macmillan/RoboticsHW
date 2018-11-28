from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np


class Problem18_1():
    def __init__(self):
        self.x_min = 0
        self.x_max = 15
        self.y_min = 0
        self.y_max = 10
        self.eta = 1
        self.start = (0, 5)
        self.goal = (15, 5)
        self.obstacles = []
        self.addObstacles()

    def addField(self, X, Y):
        return (X - self.goal[0])**2 + ((Y - self.goal[1])**2) - 1

    def addObstacles(self):
        self.obstacles.append((6, 4, 2, 20))
        self.obstacles.append((8, 6, 3, 20))

    def circle(self, x, y, c):
        """c = (cx, cy, radius, cap)"""
        val = (((x - c[0]) * (x - c[0]) / (c[2] * c[2])) +
               ((y - c[1]) * (y - c[1]) / (c[2] * c[2])) - 1)
        val[val < 0.01] = 0.01
        val = self.eta / val
        return val

    def setXYZ(self):
        X = np.arange(self.x_min - 1, self.x_max + 1, 0.1)
        Y = np.arange(self.y_min - 1, self.y_max + 1, 0.1)
        X, Y = np.meshgrid(X, Y)
        Z = self.circle(X, Y, self.obstacles[0]) +\
            self.circle(X, Y, self.obstacles[1])
        Z[Z > 80.0] = 80.0
        Z += self.addField(X, Y)
        self.Z = Z.flatten()
        self.X = X.flatten()
        self.Y = Y.flatten()

    def plotObstacles(self):
        self.setXYZ()
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        plt.gca().set_aspect('equal', adjustable='box')
        ax.plot_trisurf(self.X, self.Y, self.Z, antialiased=False)
        # ax.plot(myline)
        plt.show()


if __name__ == '__main__':
    p18 = Problem18_1()
    p18.plotObstacles()
