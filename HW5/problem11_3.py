import random
from queue import Queue
from collections import deque


class WaveFrontDemo():
    def __init__(self):
        # random.seed(a=None, version=2)
        random.seed(a=2, version=2)
        self.reset()

    def reset(self):
        self.N = 30
        self.num_obs = 4, 9
        self.obs_size = 2, 5
        self.start = 0, 0
        self.goal = 29, 29
        self.obstacles = []
        self.__generateMap()
        self.path = deque()

    def newMap(self, n=30,
               min_obs=4, max_obs=9,
               min_obs_size=2, max_obs_size=5,
               start=(0, 0), goal=(29, 29)):
        if n > 99:
            print('\n******************* WARNING *******************' +
                  '\nn is too large for newMap(). Usage n=0 to n=99' +
                  '\n***********************************************\n')
            exit()
        self.N = n
        self.num_obs = min_obs, max_obs
        self.obs_size = min_obs_size, max_obs_size
        self.start = start
        self.goal = goal if (goal[0] < n and goal[1] < n) else (n - 1, n - 1)
        self.obstacles = []
        self.__generateMap()
        self.__generateObstacles(random.randint(self.num_obs[0],
                                                self.num_obs[1]))
        self.__addObsToMap()
        self.__addStartGoal()
        self.__fillBFS()
        self.path.append(self.start)

    def __generateMap(self):
        self.grid = [['|  |' for x in range(self.N)] for y in range(self.N)]
        # self.printGrid()

    def __generateObstacles(self, obs):
        """Generate a number of obstacles of random makeup"""
        for i in range(obs):
            self.__obstacleOfChaos(n=random.randint(self.obs_size[0],
                                                    self.obs_size[1]))

    def __obstacleOfChaos(self, n=3):
        """Generates random square array of random amounts of obstacles"""
        empty = '|  |'
        obstacle = '|██|'
        # Exploiting list comprehension
        obs_obj = [[empty if bool(random.getrandbits(1)) else obstacle
                   for x in range(n)] for y in range(n)]
        self.obstacles.append(obs_obj)

    def __addObsToMap(self):
        for obs in self.obstacles:
            x = random.randint(0, self.N)
            y = random.randint(0, self.N)
            x_new = 0
            y_new = 0
            size = len(obs)

            # Ugly, but nested for loops are ugly
            for i in range(size):
                x_new = i + x
                # Bounds checking
                if x_new < self.N:
                    for j in range(size):
                        y_new = j + y
                        # Bounds checking
                        if y_new < self.N:
                            self.grid[x_new][y_new] = obs[i][j]

    def __addStartGoal(self):
        self.grid[self.start[0]][self.start[1]] = '|SS|'
        self.grid[self.goal[0]][self.goal[1]] = '|GG|'

    def __fillBFS(self):
        """Simple 4 direction BFS"""
        q = Queue(maxsize=self.N * self.N)
        q.put(self.start)
        dist = 0

        while not q.empty():
            pos = q.get()
            x = pos[0]
            y = pos[1]

            next_pos = [(x, y - 1)]       # North
            next_pos.append((x + 1, y))   # East
            next_pos.append((x, y + 1))   # South
            next_pos.append((x - 1, y))   # West

            dist = self.grid[x][y]
            dist = dist[1:3] if self.grid[x][y] != '|SS|' else '00'

            # Increase current dist for all points we are investigating
            dist = int(dist) + 1
            dist = str(dist) if dist > 9 else '0' + str(dist)
            dist = '|' + dist + '|'
            for pt in next_pos:
                if self.__inBounds(pt) and self.__notObsOrVisited(pt):
                    self.grid[pt[0]][pt[1]] = dist
                    q.put(pt)

    def __inBounds(self, pt):
        """Checks if the point is in the bounds of the map"""
        return 0 <= pt[0] < self.N and 0 <= pt[1] < self.N

    def __notObsOrVisited(self, pt):
        """Ensures not an obstacle or previously visited"""
        return self.grid[pt[0]][pt[1]] == '|  |'

    def __validPoint(self, pt):
        return self.__inBounds(pt) and self.grid[pt[0]][pt[1]] != '|██|'

    def navigate(self, point=None):
        """Returns a deque object containing the path"""
        if point is None:
            self.__navigate(self.start)
        else:
            self.__navigate(point)

        if not self.path:
            print('Unable to form a path due to obstacles')

        return self.path

    def __navigate(self, point):
        """Recursive function call to walk path"""
        print('recursing')
        x = point[0]
        y = point[1]
        pt_str = self.grid[x][y]

        # Found the end
        if pt_str == '|GG|':
            return '|GG|'

        dist = pt_str[1:3] if pt_str != '|SS|' else '00'
        dist = int(dist)

        pts = [(x, y - 1)]       # North
        pts.append((x + 1, y))   # East
        pts.append((x, y + 1))   # South
        pts.append((x - 1, y))   # West

        found = ''
        for pt in pts:
            if self.__validPoint(pt) and found == '':
                pt_str = self.grid[pt[0]][pt[1]]
                if pt_str == '|GG|':
                    return '|GG|'
                next_dist = int(pt_str[1:3])
                if next_dist == dist + 1:
                    self.path.append(pt)
                    found = self.__navigate(pt)

        if found == '':
            self.path.pop()
            return ''
        else:
            return '|GG|'

    def printGrid(self):
        for i in range(self.N):
            row = ''
            for j in range(self.N):
                row += self.grid[i][j]
            print(row)
        print()

    def __printObstacle(self, obs):
        for i in range(3):
            row = ''
            for j in range(3):
                row += obs[i][j]
            print(row)

    def drawPath(self):
        while self.path:
            x, y = self.path.pop()
            self.grid[x][y] = '|--|'
        self.grid[self.start[0]][self.start[1]] = '|SS|'


if __name__ == "__main__":
    wf = WaveFrontDemo()
    wf.newMap(n=15)
    path = wf.navigate()

    wf.printGrid()

    wf.drawPath()
    wf.printGrid()
