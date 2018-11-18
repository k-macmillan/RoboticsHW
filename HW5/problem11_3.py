import numpy as np


class WaveFrontDemo():
    def __init__(self):
        self.N = 30
        self.grid = np.empty([self.N, self.N], dtype=np.unicode_)
        # self.mark = '\u2588'
        # self.mark = self.mark.encode('utf-8')
        self.obstacles = []
        self.__generateMap()
        self.__generateObstacles()

    def __generateMap(self):
        self.grid[:] = ' '
        self.printGrid()

    def __generateObstacles(self):
        self.__obstacle1()
        self.__obstacle2()
        self.__obstacle3()
        # Apply each obstacle to the map at a random location

    def __obstacle1(self):
        obstacle = np.array([[' ', '█', ' '],
                             [' ', '█', '█'],
                             ['█', '█', ' ']])
        self.obstacles.append(obstacle)
        # self.__printObstacle(obstacle)

    def __obstacle2(self):
        obstacle = np.array([[' ', ' ', ' '],
                             [' ', ' ', ' '],
                             [' ', ' ', ' ']])
        self.obstacles.append(obstacle)
        # self.__printObstacle(obstacle)

    def __obstacle3(self):
        obstacle = np.array([[' ', ' ', ' '],
                             [' ', ' ', ' '],
                             [' ', ' ', ' ']])
        self.obstacles.append(obstacle)
        # self.__printObstacle(obstacle)

    def __agentOfChaos(self, n=3):
        """Generates random square array of random amounts of obstacles"""
        pass

    def __discretizeMap(self):
        pass

    def __markObstacles(self):
        obstacle = '█'
        print(obstacle)
        pass

    def navigate(self):
        pass

    def printGrid(self):
        for i in range(self.N):
            row = ""
            for j in range(self.N):
                row += self.grid[i][j]
            print(row)

    def __printObstacle(self, obs):
        for i in range(3):
            row = ""
            for j in range(3):
                row += obs[i][j]
            print(row)


if __name__ == "__main__":
    wf = WaveFrontDemo()
    wf.navigate()
