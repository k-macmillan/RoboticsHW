from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


class Point():
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __str__(self):
        return '{},{},{}'.format(self.x, self.y, self.z)

    def printPoint(self):
        print('{},{},{}'.format(self.x, self.y, self.z))


class Beacon():
    def __init__(self, x=0, y=0, z=0, d=0, color='b'):
        self.c = Point(x, y, z)
        self.r = d
        self.color = color
        self.E = float("inf")
        self.__setMinMax()
        self.__addToPlot()
        # self.printAttribs()

    def dist(self, b):
        return sqrt((self.x - b.x) * (self.x - b.x) +
                    (self.y - b.y) * (self.y - b.y) +
                    (self.z - b.z) * (self.z - b.z))

    def __addToPlot(self):
        ax.scatter(self.c.x, self.c.y, self.c.z, c=self.color)
        # Make data
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)
        x = self.r * np.outer(np.cos(u), np.sin(v)) + self.c.x
        y = self.r * np.outer(np.sin(u), np.sin(v)) + self.c.y
        z = self.r * np.outer(np.ones(np.size(u)), np.cos(v)) + self.c.z
        ax.plot_wireframe(x, y, z, color=self.color)

    def calcError(self, x, y, z):
        self.E = np.abs(np.sqrt(x * x + y * y + z * z) - self.r)

    def onBlock(self, a, b):
        """ This is completely wrong and requires overhaul
        """
        if (((a.x <= self.x_min <= b.x) or (a.x <= self.x_max <= b.x)) and
           ((a.y <= self.y_min <= b.y) or (a.y <= self.y_max <= b.y)) and
           ((a.z <= self.z_min <= b.z) or (a.z <= self.z_max <= b.z))):
            return True
        return False

    def __setMinMax(self):
        self.x_min = self.c.x - self.r
        self.x_max = self.c.x + self.r
        self.y_min = self.c.y - self.r
        self.y_max = self.c.y + self.r
        self.z_min = self.c.z - self.r
        self.z_max = self.c.z + self.r

    def printAttribs(self):
        print('Beacon attribs:')
        print(self.color)
        print(self.x_min)
        print(self.x_max)
        print(self.y_min)
        print(self.y_max)
        print(self.z_min)
        print(self.z_max)
        print()


class Plot():
    def __init__(self):
        self.beacons = []
        self.x_min = "inf"
        self.x_max = "-inf"
        self.y_min = "inf"
        self.y_max = "-inf"
        self.z_min = "inf"
        self.z_max = "-inf"
        self.a = Point()
        self.b = Point()
        self.depth = 0

    def addBeacon(self, b):
        self.beacons.append(b)
        # Running this every time since there shouldn't be that many beacons
        self.__fillMinMax()

    def __fillMinMax(self):
        """ Quick and dirty fill of min/max, not efficient """
        self.x_min = min(point.c.x for point in self.beacons)
        self.x_max = max(point.c.x for point in self.beacons)
        self.y_min = min(point.c.y for point in self.beacons)
        self.y_max = max(point.c.y for point in self.beacons)
        self.z_min = min(point.c.z for point in self.beacons)
        self.z_max = max(point.c.z for point in self.beacons)
        self.__setPoints()

    def __setPoints(self):
        self.a.x = self.x_min
        self.a.y = self.y_min
        self.a.z = self.z_min
        self.b.x = self.x_max
        self.b.y = self.y_max
        self.b.z = self.z_max

    def printAttribs(self):
        print(self.x_min)
        print(self.x_max)
        print(self.y_min)
        print(self.y_max)
        print(self.z_min)
        print(self.z_max)
        a.printPoint()
        b.printPoint()

    def findRobot(self):
        self.depth = 0
        return self.__checkGrids((self.a, self.b), self.beacons)

    def __checkGrids(self, pts, beacons):
        """ Recursive function that checks 4 quadrants for a circle
            intersection. If an intersection is found it counts that up. After
            all intersections have been counted the section with the most is
            dove into and the process repeats until it finds beacons - 1
            intersections.
        """

        # For clarity:
        a = Point(pts[0].x, pts[0].y, pts[0].z)
        b = Point(pts[1].x, pts[1].y, pts[1].z)

        grid = [0] * 4
        x_mid = ((b.x - a.x) / 2.0) + a.x
        y_mid = ((b.y - a.y) / 2.0) + a.y
        ab_list = []
        ab_list.append((a, Point(x_mid, y_mid, b.z)))
        ab_list.append((Point(a.x, y_mid, a.z), Point(x_mid, b.y, b.z)))
        ab_list.append((Point(x_mid, a.y, a.z), Point(b.x, y_mid, b.z)))
        ab_list.append((Point(x_mid, y_mid, a.z), b))

        for asdf in ab_list:
            print(str(asdf[0]) + ' ' + str(asdf[1]))
        # exit()
        for b in beacons:
            # Check each block
            for i in range(4):
                if b.onBlock(ab_list[i][0], ab_list[i][1]):
                    grid[i] += 1

        max_indx = grid.index(max(grid))
        best_pts = pts
        if grid[max_indx] == len(beacons) and self.depth < 10:
            self.depth += 1
            base_check = self.__checkGrids(ab_list[max_indx], beacons)
            if base_check is None:
                return best_pts

        if best_pts == pts and self.depth < 10:
            return None
        else:
            return best_pts


def sanityCheck(land_plot, ax):
    land_plot.addBeacon(Beacon(0, 0, 0, 10, color='r'))
    land_plot.addBeacon(Beacon(20, 0, 0, 10, color='g'))
    land_plot.addBeacon(Beacon(0, 20, 0, 10, color='b'))
    land_plot.addBeacon(Beacon(0, 0, 20, 9.75, color='c'))
    # land_plot.addBeacon(Beacon(967, 653, 46, 529, color='c'))  # this one?
    # land_plot.addBeacon(Beacon(593, 186, 989, 610, color='m'))
    # land_plot.printAttribs()
    ax.set_xlim(-5, 25)
    ax.set_ylim(-5, 25)
    ax.set_zlim(-5, 25)
    pts = land_plot.findRobot()

    if pts is not None:
        print('\n\nRobot at:')
        pts[0].printPoint()
        pts[1].printPoint()


if __name__ == '__main__':
    sanity = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    land_plot = Plot()
    if sanity:
        sanityCheck(land_plot, ax)
    else:
        land_plot.addBeacon(Beacon(884, 554, 713, 222, color='r'))
        land_plot.addBeacon(Beacon(120, 703, 771, 843, color='g'))
        land_plot.addBeacon(Beacon(938, 871, 583, 436, color='b'))
        land_plot.addBeacon(Beacon(967, 653, 46, 529, color='c'))  # this one?
        land_plot.addBeacon(Beacon(593, 186, 989, 610, color='m'))

        # land_plot.printAttribs()
        # print(land_plot.findRobot()

        ax.set_xlim(-200, 1200)
        ax.set_ylim(-200, 1200)
        ax.set_zlim(-200, 1200)
    ax.set_aspect('equal')
    plt.xlabel('x position')
    plt.ylabel('y position')

    plt.show()
    fig.savefig('problem9-1.pdf',
                format='pdf',
                dpi=1200)
