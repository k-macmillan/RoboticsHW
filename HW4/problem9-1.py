from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import queue


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
        return '({}, {}, {})'.format(self.x, self.y, self.z)

    def midpoint(self, other):
        return Point((self.x + other.x) / 2.0,
                     (self.y + other.y) / 2.0,
                     (self.z + other.z) / 2.0)

    def distSquared(self, other):
        return (self.x - other.x) * (self.x - other.x) +\
               (self.y - other.y) * (self.y - other.y) +\
               (self.z - other.z) * (self.z - other.z)

    def printPoint(self):
        print(str(self))


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
        u = np.linspace(0, 2 * np.pi, 60)
        v = np.linspace(0, np.pi, 60)
        x = self.r * np.outer(np.cos(u), np.sin(v)) + self.c.x
        y = self.r * np.outer(np.sin(u), np.sin(v)) + self.c.y
        z = self.r * np.outer(np.ones(np.size(u)), np.cos(v)) + self.c.z
        ax.plot_wireframe(x, y, z, color=self.color)

    def calcError(self, pt):
        self.E = np.abs(np.sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) -
                        self.r)
        return self.E

    def onBlock(self, a, b):
        """ Detects if this block (a,b) is within the beacon's radius
            https://yal.cc/rectangle-circle-intersection-test/
        """
        dx = self.c.x - max(a.x, min(self.c.x, b.x))
        dy = self.c.y - max(a.y, min(self.c.y, b.y))
        dz = self.c.z - max(a.z, min(self.c.z, b.z))
        return (dx * dx + dy * dy + dz * dz) <= (self.r * self.r)

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
        self.b_len = 0
        self.x_min = "inf"
        self.x_max = "-inf"
        self.y_min = "inf"
        self.y_max = "-inf"
        self.z_min = "inf"
        self.z_max = "-inf"
        self.max_depth = 0
        self.max_checks = 0
        self.a = Point()
        self.b = Point()
        self.depth = 0
        self.robot_loc = Point()
        self.q = queue.Queue()

    def addBeacon(self, b):
        self.beacons.append(b)
        # Running this every time since there shouldn't be that many beacons
        self.__fillMinMax()
        self.b_len = len(self.beacons)

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
        max_len = max(self.x_max - self.x_min,
                      self.y_max - self.y_min,
                      self.z_max - self.z_min)
        if max_len > 0:
            self.max_depth = log2(max_len) + 2

    def centerPoint(self, pt):
        return Point(((pt[1].x - pt[0].x) / 2.0) + pt[0].x,
                     ((pt[1].y - pt[0].y) / 2.0) + pt[0].y,
                     ((pt[1].z - pt[0].z) / 2.0) + pt[0].z)

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
        best_pts = []
        best_depth = []
        self.q.put(((self.a, self.b), 0))
        while not self.q.empty():
            pts, depth = self.q.get()
            # print('\nab: {}   {}'.format(str(pts[0]), str(pts[1])))
            # print('Depth: ', depth)
            # exit()
            self.__checkGrids(pts, depth)
            best_pts.append(pts)
            best_depth.append(depth)

        # Find the index that made it deepest
        idx = best_depth.index(max(best_depth))
        midp = best_pts[idx]
        print('\nmidp: {}   {}'.format(str(midp[0]), str(midp[1])))
        print('Depth: ', best_depth[idx])
        print('Runs : ', len(best_depth))
        count = 0
        for i in range(len(best_depth)):
            if best_depth[i] == best_depth[idx]:
                count += 1

        print('At depth: ', count)

        if midp[0] is not None:
            self.robot_loc = midp[0].midpoint(midp[1])
            print('\nRobot at:')
            self.robot_loc.printPoint()
            E = 0.0
            for b in self.beacons:
                E += b.calcError(self.robot_loc)
                print('Beacon error: ', b.E)
            print('With an error of: ', E)
        else:
            print('\nRobot not found!')

    def __checkGrids(self, pts, depth):
        """ Recursive function that checks 4 quadrants for a circle
            intersection. If an intersection is found it counts that up. After
            all intersections have been counted the section with the most is
            dove into and the process repeats until it finds beacons - 1
            intersections.
        """
        # print('\nab: {}   {}'.format(str(pts[0]), str(pts[1])))

        # For clarity:
        a = Point(pts[0].x, pts[0].y, pts[0].z)
        b = Point(pts[1].x, pts[1].y, pts[1].z)
        quadrants = 8

        grid = [0] * quadrants
        x_mid = ((b.x - a.x) / 2.0) + a.x
        y_mid = ((b.y - a.y) / 2.0) + a.y
        z_mid = ((b.z - a.z) / 2.0) + a.z
        ab_list = []

        # This was not easy to ensure no fat fingering
        ab_list.append((a, Point(x_mid, y_mid, z_mid)))
        ab_list.append((Point(a.x, y_mid, a.z), Point(x_mid, b.y, z_mid)))
        ab_list.append((Point(x_mid, a.y, a.z), Point(b.x, y_mid, z_mid)))
        ab_list.append((Point(x_mid, y_mid, a.z), Point(b.x, b.y, z_mid)))

        ab_list.append((Point(a.x, a.y, z_mid), Point(x_mid, y_mid, b.z)))
        ab_list.append((Point(a.x, y_mid, z_mid), Point(x_mid, b.y, b.z)))
        ab_list.append((Point(x_mid, a.y, z_mid), Point(b.x, y_mid, b.z)))
        ab_list.append((Point(x_mid, y_mid, z_mid), b))

        # Fill grid with hits
        for b in self.beacons:
            # Check each block
            for i in range(quadrants):
                if b.onBlock(ab_list[i][0], ab_list[i][1]):
                    grid[i] += 1

        # For each block with len(beacons) find the closest
        # min_dist = float("inf")
        # min_idx = -1
        # for i in range(quadrants):
        #     if grid[i] == len(self.beacons):
        #         # Calculate center distances
        #         cp = self.centerPoint(ab_list[i])
        #         dist = 0.0
        #         for b in self.beacons:
        #             dist += cp.distSquared(b.c)
        #         print('{} dist: {}'.format(i, dist))
        #         if dist < min_dist:
        #             min_dist = dist
        #             min_idx = i

        # print('ab_list:')
        # for i in range(len(ab_list)):
        #     print(str(grid[i]) + ': ' +
        #           str(ab_list[i][0]) +
        #           '   ' +
        #           str(ab_list[i][1]))

        # if depth < self.max_depth and min_idx != -1:
        #     ret_val = self.__checkGrids(ab_list[min_idx], depth + 1)
        # else:
        #     ret_val = (None, pts)

        # if ret_val[0] is None:
        #     return (depth, pts)
        # else:
        #     return ret_val

        # best = []
        # depth_ary = []
        # for i in range(quadrants):
        #     if grid[i] == len(self.beacons) and\
        #        depth < self.max_depth:  # and self.max_checks > 0:
        #         ret_val = self.__checkGrids(ab_list[i], depth + 1)
        #         # If we hit no more beacon overlaps
        #         if ret_val[1] is None:
        #             # print('Depth: ', depth)
        #             # pts[0].printPoint()
        #             # pts[1].printPoint()
        #             best.append(pts)
        #             depth_ary.append(depth)
        #         else:
        #             depth_ary.append(ret_val[0])
        #             best.append(ret_val[1])

        # if len(best) == 0:
        #     return (depth, None)
        # else:
        #     idx = depth_ary.index(max(depth_ary))
        #     return (depth_ary[idx], best[idx])

        found = 0
        for i in range(self.b_len):
            if grid[i] == self.b_len:
                found += 1

        # print('Found: ', found)
        # Means we are not in a solution
        if found == quadrants:
            return (None, None)

        for i in range(quadrants):
            if grid[i] == self.b_len and depth < self.max_depth:
                self.q.put((ab_list[i], depth + 1))


def sanityCheck2D(land_plot, ax, fig):
    """ Essentially a unit test that assumes the drone is at:
        x = 2.32
        y = 12.824
        Should print those same points
    """
    land_plot.addBeacon(Beacon(0, 0, 0, 13.032166973, color='r'))
    land_plot.addBeacon(Beacon(20, 0, 0, 21.841185316, color='g'))
    land_plot.addBeacon(Beacon(0, 20, 0, 7.54170909, color='b'))
    land_plot.addBeacon(Beacon(20, 20, 0, 19.080811723, color='c'))
    ax.set_xlim(-5, 25)
    ax.set_ylim(-5, 25)
    ax.set_zlim(-5, 25)
    land_plot.findRobot()
    makeplot(fig, 'problem9-1_2D.pdf')


def sanityCheck3D(land_plot, ax, fig):
    """ Essentially a unit test that assumes the drone is at:
        x = 2.32
        y = 12.824
        z = 11.68
        Should print those same points
    """
    # land_plot.addBeacon(Beacon(0, 0, 0, 17.500279312, color='r'))
    # land_plot.addBeacon(Beacon(20, 0, 0, 24.768120155, color='g'))
    # land_plot.addBeacon(Beacon(0, 20, 0, 13.903228977, color='b'))
    # land_plot.addBeacon(Beacon(20, 20, 0, 22.371852315, color='c'))
    # land_plot.addBeacon(Beacon(20, 20, 20, 20.815853958, color='k'))
    land_plot.addBeacon(Beacon(0, 0, 0, 20.784609691, color='k'))
    land_plot.addBeacon(Beacon(20, 20, 20, 13.856406461, color='r'))
    ax.set_xlim(-5, 25)
    ax.set_ylim(-5, 25)
    ax.set_zlim(-5, 25)
    land_plot.findRobot()
    makeplot(fig, 'problem9-1_3D.pdf')


def makeplot(fig, saveas=""):
        plt.show()
        if saveas != "":
            fig.savefig(saveas,
                        format='pdf',
                        dpi=1200)
        plt.gcf().clear()
        fig = plt.figure()


if __name__ == '__main__':
    sanity = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
    plt.xlabel('x position')
    plt.ylabel('y position')

    if sanity:
        # sanityCheck2D(Plot(), ax, fig)
        sanityCheck3D(Plot(), ax, fig)
    else:
        land_plot = Plot()
        land_plot.addBeacon(Beacon(884, 554, 713, 222, color='r'))
        land_plot.addBeacon(Beacon(120, 703, 771, 843, color='g'))
        land_plot.addBeacon(Beacon(938, 871, 583, 436, color='b'))
        land_plot.addBeacon(Beacon(967, 653, 46, 529, color='c'))  # this one?
        land_plot.addBeacon(Beacon(593, 186, 989, 610, color='m'))

        # land_plot.printAttribs()
        land_plot.findRobot()

        ax.set_xlim(-200, 1200)
        ax.set_ylim(-200, 1200)
        ax.set_zlim(-200, 1200)

        plt.show()
        fig.savefig('problem9-1.pdf',
                    format='pdf',
                    dpi=1200)
