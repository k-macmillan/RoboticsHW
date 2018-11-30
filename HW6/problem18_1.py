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
        self.g = 0.0175
        self.start = (0, 5)
        self.goal = (15, 5)
        self.obstacles = []
        self.addObstacles()
        self.avg = 0

    # Add field and obstacles
    def addField(self, X, Y):
        return (X - self.goal[0])**2 + ((Y - self.goal[1])**2) - 1

    def addObstacles(self):
        self.obstacles.append((6, 4, 2))
        self.obstacles.append((8, 6, 3))

    def circle(self, x, y, c):
        """c = (cx, cy, radius, cap)"""
        val = (((x - c[0])**2 / (c[2] * c[2])) +
               ((y - c[1])**2 / (c[2] * c[2])) - 1)
        val[val < 0.001] = 0.001
        val = 1 / val
        # 1 / (((x - 8)^2 / 9) + ((y - 6)^2 / 9) - 1)
        return val

    # Partial derivative section
    def diffFieldX(self, x):
        return 2 * x - 30

    def diffFieldY(self, y):
        return 2 * y - 10

    def diffObstacle1X(self, x, y):
        top = 8 * x - 8 * 6
        bot = (x**2 - 12 * x + y**2 + 48 - 8 * y)**2
        return -(top / bot)

    def diffObstacle2X(self, x, y):
        top = 18 * x - 9 * 16
        bot = (x**2 - 16 * x + y**2 + 91 - 12 * y)**2
        return -(top / bot)

    def diffObstacle1Y(self, x, y):
        top = 8 * y - 8 * 4
        bot = (x**2 - 12 * x + y**2 + 48 - 8 * y)**2
        return -(top / bot)

    def diffObstacle2Y(self, x, y):
        top = 18 * y - 18 * 6
        bot = (x**2 - 16 * x + y**2 + 91 - 12 * y)**2
        return -(top / bot)

    # Run descent and graph 2D or 3D
    def setDXDYDZ(self):
        self.setXYZ()
        loops = len(self.Z)
        x = [self.start[0]]
        y = [self.start[1]]
        done = False
        for i in range(1, loops):
            dist = (x[-1] - self.goal[0])**2 + (y[-1] - self.goal[1])**2

            if dist > self.eta:
                self.g = self.eta / dist
            elif dist > 0.01:
                self.g *= 0.9
            else:
                if not done:
                    print('Took {} iterations.'.format(i))
                    done = True
                self.g = 0

            # Calc partial with respect to x
            dx = (self.diffFieldX(x[-1]) +
                  self.diffObstacle1X(x[-1], y[-1]) +
                  self.diffObstacle2X(x[-1], y[-1]))

            # Calc partial with respect to y
            dy = (self.diffFieldY(y[-1]) +
                  self.diffObstacle1Y(x[-1], y[-1]) +
                  self.diffObstacle2Y(x[-1], y[-1]))
            dx = self.g * dx
            dy = self.g * dy

            x.append(x[-1] - dx)
            y.append(y[-1] - dy)

        # 'x' because it makes the code cleaner, then assigned to self.x
        self.x = x
        self.y = y

    def plotObstacles2D(self):
        self.setDXDYDZ()
        ax = plt.gca()
        plt.gca().set_aspect('equal', adjustable='box')
        ax.plot(self.x, self.y)
        for x, y, r in self.obstacles:
            ax.add_artist(plt.Circle((x, y), r, color='r'))
        ax.add_artist(plt.Circle((self.start[0], self.start[1]),
                                 radius=0.125, color='g'))
        ax.add_artist(plt.Circle((self.goal[0], self.goal[1]),
                                 radius=0.125, color='g'))
        plt.xlim(-1, 16)
        plt.ylim(-1, 11)
        plt.show()

    def setXYZ(self):
        X = np.arange(self.x_min - 1, self.x_max + 1, 0.1)
        Y = np.arange(self.y_min - 1, self.y_max + 1, 0.1)
        X, Y = np.meshgrid(X, Y)
        Z = self.circle(X, Y, self.obstacles[0]) +\
            self.circle(X, Y, self.obstacles[1])
        Z[Z > 150.0] = 150.0
        Z += self.addField(X, Y)
        self.Z = Z.flatten()
        self.X = X.flatten()
        self.Y = Y.flatten()

    def plotObstacles3D(self):
        # self.setXYZ()
        self.setDXDYDZ()
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        plt.gca().set_aspect('equal', adjustable='box')
        # ax.plot_trisurf(self.X, self.Y, self.Z, antialiased=False)
        ax.plot(self.x, self.y, self.Z, 'go')
        plt.xlim(-1, 16)
        plt.ylim(-1, 11)
        plt.show()


if __name__ == '__main__':
    p18 = Problem18_1()
    p18.plotObstacles3D()
    # p18.plotObstacles2D()
