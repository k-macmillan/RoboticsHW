import random
from queue import Queue


class WaveFrontDemo():
    def __init__(self):
        random.seed(a=None, version=2)
        self.reset()

    def reset(self):
        self.N = 30
        self.num_obs = 4, 9
        self.obs_size = 2, 5
        self.start = 0, 0
        self.goal = 29, 29
        self.obstacles = []
        self.__generateMap()

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
            size = len(obs)
            # Ugly, but nested for loops are ugly
            for i in range(size):
                x_new = i + x
                if x_new < self.N:
                    for j in range(size):
                        y_new = j + y
                        if y_new < self.N:
                            self.grid[x_new][y_new] = obs[i][j]

    def __addStartGoal(self):
        self.grid[self.start[0]][self.start[1]] = '|SS|'
        self.grid[self.goal[0]][self.goal[1]] = '|GG|'

    def __fillBFS(self):
        """Simple 4 direction BFS"""
        q = Queue(maxsize=self.N * self.N)
        q.put(self.start)
        depth = 0

        while not q.empty():
            pos = q.get()
            x = pos[0]
            y = pos[1]

            next_pos = [(x, y - 1)]       # North
            next_pos.append((x + 1, y))   # East
            next_pos.append((x, y + 1))   # South
            next_pos.append((x - 1, y))   # West

            depth = self.grid[x][y]
            depth = depth[1:3] if self.grid[x][y] != '|SS|' else '00'

            # Increase current depth for all points we are investigating
            depth = int(depth) + 1
            depth = str(depth) if depth > 9 else '0' + str(depth)
            depth = '|' + depth + '|'
            for pt in next_pos:
                if self.__validPoint(pt):
                    self.grid[pt[0]][pt[1]] = depth
                    q.put(pt)

    def __validPoint(self, pt):
        if 0 <= pt[0] < self.N and 0 <= pt[1] < self.N:
            return self.__notObstacle(pt)

    def __notObstacle(self, pt):
        """Ensures not an obstacle"""
        return self.grid[pt[0]][pt[1]] == '|  |'

    def navigate(self):
        self.__fillBFS()

    def printGrid(self):
        for i in range(self.N):
            row = ''
            for j in range(self.N):
                row += self.grid[i][j]
            print(row)

    def __printObstacle(self, obs):
        for i in range(3):
            row = ''
            for j in range(3):
                row += obs[i][j]
            print(row)


if __name__ == "__main__":
    wf = WaveFrontDemo()
    wf.newMap(15)
    # wf.printGrid()
    wf.navigate()
    # print()
    wf.printGrid()
