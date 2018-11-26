import numpy as np
import matplotlib.pyplot as plt


class problem17_4a():
    def __init__(self):
        self.w1 = 3
        self.w2 = 3
        self.r = 5
        self.L = 12
        self.dt = 0.2
        self.t = np.arange(0, 16 + self.dt, self.dt) # 81
        self.N = len(self.t)
        self.rdt = self.r * self.dt / 2.0
        self.V = np.array([[0.05, 0.02, 0.01],
                           [0.02, 0.05, 0.01],
                           [0.01, 0.01, 0.10]])
        self.W = np.array([[0.08, 0.02],
                           [0.02, 0.07]])
        self.xhat0 = np.array([[0.0, 0.0, 0.0]])
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 0]])
        self.HT = self.H.T
        self.P = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]])
        self.I = np.identity(3)
        self.phy_noise = np.random.multivariate_normal([0, 0, 0], self.V, self.N)
        self.obs_noise = np.random.multivariate_normal([0, 0], self.W, self.N)

    def avgWS(self):
        return self.rdt * (self.w1 + self.w2)

    # Specific functions to the problem
    def updateXhat0(self):
        """State update"""
        avg_ws = self.avgWS()

        # Updates
        x_n = self.xhat0[0][0] + avg_ws * np.cos(self.xhat0[0][2])
        y_n = self.xhat0[0][1] + avg_ws * np.sin(self.xhat0[0][2])
        theta_n = self.xhat0[0][2] + (self.rdt / self.L) * (self.w1 - self.w2)
        self.xhat0 = np.array([[x_n, y_n, theta_n]])

    def updateF(self):
        avg_ws = self.avgWS()
        self.F = np.array([[1.0, 0.0, -avg_ws * np.sin(self.xhat0[0][2])],
                           [0.0, 1.0,  avg_ws * np.cos(self.xhat0[0][2])],
                           [0.0, 0.0, 1.0]])

    def setWS(self, t):
        """Given a time, return L, R wheelspeeds"""
        if t <= 5.0:
            self.w1 = 3
            self.w2 = 3
        elif t <= 6.0:
            self.w1 = 1
            self.w2 = -1
        elif t <= 10.0:
            self.w1 = 3
            self.w2 = 3
        elif t <= 11.0:
            self.w1 = -1
            self.w2 = 1
        else:  #  t <= 16.0
            self.w1 = 3
            self.w2 = 3

    def updateP0(self):
        self.P = np.matmul(np.matmul(self.F, self.P), self.F.T) + self.V

    def updateK(self):
        HP = np.matmul(self.H, self.P)
        HPHT = np.matmul(HP, self.HT)[:2, :2]  # Strip zeroes from HPHT
        PHT = np.matmul(self.P, self.HT)
        invHPHTW = np.linalg.inv(HPHT + self.W)
        # Pad zeros back into HPHTW
        self.K = np.matmul(PHT, self.padInv(invHPHTW, self.I))

    def padInv(self, a, b):
        """numpy.pad would have been nice but I don't have time to figure it out
           Pads a into the shape of b
        """
        ret_val = np.zeros(b.shape)
        ret_val[:a.shape[0],:a.shape[1]] = a
        return ret_val

    def updateXhat1(self, z):
        self.xhat1 = self.xhat0 + np.matmul(self.K, (z - self.xhat0).T)

    def updateP1(self):
        self.P = np.matmul(self.I - np.matmul(self.K, self.H), self.P)

    def run(self):
        # Plot arrays
        actual = np.zeros((self.N, 3))
        predict = np.zeros((self.N, 3))
        obs = np.zeros((self.N, 3))

        # For each time in t
        for i in range(self.N):
            # Set wheel speed given time
            self.setWS(self.t[i])

            # Predict State
            self.updateXhat0()
            self.xhat0 = np.add(self.xhat0, self.phy_noise[i])  # Verified working
            # print(self.xhat0)
            actual[i] = self.xhat0[0]

            # Predict estimate covariance
            self.updateF()
            self.updateP0()

            # Optimal Kalman gain
            self.updateK()

            # Update state estimate
            z = self.xhat0
            z[0][0] += self.obs_noise[i][0]
            z[0][1] += self.obs_noise[i][1]
            self.updateXhat1(z)

            # Update estimate covariance
            self.updateP1()

            # Add to plot arrays
            predict[i] = self.xhat1[0]
            obs[i] = z

        # Plot data points
        fig = plt.figure()
        plt.plot(obs[:,0], obs[:,1], 'r.', label='Observed')
        plt.plot(predict[:,0], predict[:,1], 'g-', label='Predicted')
        plt.plot(actual[:,0], actual[:,1], 'b-', label='Actual')
        plt.ylabel('$x$')
        plt.xlabel('$t$')
        plt.legend()
        fig.savefig('problem17_4a_x',
                    format='pdf',
                    dpi=1200)
        plt.show()


if __name__ == '__main__':
    p17_4a = problem17_4a()
    p17_4a.run()
