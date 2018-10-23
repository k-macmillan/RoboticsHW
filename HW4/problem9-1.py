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
        self.r_sqr = self.r * self.r
        self.color = color
        self.__addToPlot()

    def __addToPlot(self):
        ax.scatter(self.c.x, self.c.y, self.c.z, c=self.color)
        # Make spheres
        u = np.linspace(0, 2 * np.pi, 60)
        v = np.linspace(0, np.pi, 60)
        x = self.r * np.outer(np.cos(u), np.sin(v)) + self.c.x
        y = self.r * np.outer(np.sin(u), np.sin(v)) + self.c.y
        z = self.r * np.outer(np.ones(np.size(u)), np.cos(v)) + self.c.z
        ax.plot_wireframe(x, y, z, color=self.color)

    def __getCorners(self, a, b):
        points = [a]
        points.append(Point(a.x, b.y, a.z))  # Top Front Left
        points.append(Point(b.x, a.y, a.z))  # Bottom Front Right
        points.append(Point(b.x, b.y, a.z))  # Top Front Right
        points.append(Point(a.x, a.y, b.z))  # Bottom Back Left
        points.append(Point(a.x, b.y, b.z))  # Top Back Left
        points.append(Point(b.x, a.y, b.z))  # Bottom Back Right
        points.append(b)

        return points

    def onBlock(self, a, b):
        """Detects if this block (a,b) is intersected by the beacon's radius"""
        pts_in_circle = 0
        pts = self.__getCorners(a, b)
        for p in pts:
            if p.distSquared(self.c) <= self.r_sqr:
                pts_in_circle += 1
        return 0 < pts_in_circle < 8


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
        self.a = Point()
        self.b = Point()
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
            self.max_depth = log2(max_len) * 2

    def findRobot(self):
        best_pts = []
        best_depth = []
        self.q.put(((self.a, self.b), 0))
        while not self.q.empty():
            pts, depth = self.q.get()
            self.__checkGrids(pts, depth)
            best_pts.append(pts)
            best_depth.append(depth)

        centroid = Point()
        # print('Total recursions: ', len(best_depth))

        # Find the index that made it deepest and calculate the centroid
        idx = best_depth.index(max(best_depth))
        count = 0.0
        for i in range(len(best_depth)):
            if best_depth[i] == best_depth[idx]:
                centroid += best_pts[i][0].midpoint(best_pts[i][1])
                count += 1.0
        # print('count at depth: ', count)

        if centroid != Point(0, 0, 0):
            centroid.x = centroid.x / count
            centroid.y = centroid.y / count
            centroid.z = centroid.z / count

            self.robot_loc = centroid
            print('\nRobot at: {}'.format(str(self.robot_loc)))
        else:
            print('\nRobot not found!')

    def __checkGrids(self, pts, depth):
        """ Recursive function that checks 4 quadrants for a circle
            intersection. If an intersection is found it counts that up. After
            all intersections have been counted the section with the most is
            dove into and the process repeats until it finds beacons - 1
            intersections.
        """
        # For clarity:
        a = Point(pts[0].x, pts[0].y, pts[0].z)
        b = Point(pts[1].x, pts[1].y, pts[1].z)
        quadrants = 8

        grid = [0] * quadrants
        x_mid = ((b.x + a.x) / 2.0)
        y_mid = ((b.y + a.y) / 2.0)
        z_mid = ((b.z + a.z) / 2.0)
        ab_list = []

        # The 8 quadrants of pts a & b
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
        found = 0
        for i in range(self.b_len):
            if grid[i] == self.b_len:
                found += 1

        if found == quadrants:
            return

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
    land_plot.addBeacon(Beacon(0, 0, 0, 17.500279312, color='r'))
    land_plot.addBeacon(Beacon(20, 0, 0, 24.768120155, color='g'))
    land_plot.addBeacon(Beacon(0, 20, 0, 13.903228977, color='b'))
    land_plot.addBeacon(Beacon(20, 20, 0, 22.371852315, color='c'))
    land_plot.addBeacon(Beacon(20, 20, 20, 20.815853958, color='k'))
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


def problem9(land_plot, ax, fig):
    """Runs the beacon problem from the homework"""
    land_plot.addBeacon(Beacon(884, 554, 713, 222, color='r'))
    land_plot.addBeacon(Beacon(120, 703, 771, 843, color='g'))
    land_plot.addBeacon(Beacon(938, 871, 583, 436, color='b'))
    land_plot.addBeacon(Beacon(967, 653, 46, 529, color='c'))
    land_plot.addBeacon(Beacon(593, 186, 989, 610, color='m'))
    land_plot.findRobot()

    ax.set_xlim(-200, 1200)
    ax.set_ylim(-200, 1200)
    ax.set_zlim(-200, 1200)

    plt.show()
    fig.savefig('problem9-1.pdf',
                format='pdf',
                dpi=1200)


if __name__ == '__main__':
    sanity = False

    # Setup figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
    plt.xlabel('x position')
    plt.ylabel('y position')

    # Run beacon problems
    if sanity:
        # sanityCheck2D(Plot(), ax, fig)
        sanityCheck3D(Plot(), ax, fig)
    else:
        problem9(Plot(), ax, fig)
