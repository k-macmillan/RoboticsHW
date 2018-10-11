import numpy as np
from math import *
from scipy import linalg
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt


class diffdrive():
    def __init__(self):
        self.reset()
        self.plot = plt.figure()

    def reset(self):
        self.xc = 0.0
        self.yc = 0.0
        self.qc = 0.0
        self.t = 0.0
        self.dt = 0.01

        # Robot Variables
        self.r = 20
        self.L = 12
        self.w1 = 0.25
        self.w2 = 0.5
        self.x = []
        self.y = []
        self.noise_x = []
        self.noise_y = []

    def ddstep(self, xc, yc, qc, r, l, dt, w1, w2):
        xn = xc + (r * dt / 2.0) * (w1 + w2) * np.cos(qc)
        yn = yc + (r * dt / 2.0) * (w1 + w2) * np.sin(qc)
        qn = qc + (r * dt / (2.0 * l)) * (w1 - w2)
        return (xn, yn, qn)

    def a(self, plot=True):
        self.reset()
        while (self.t < 5.0 - self.dt):
            self.xc, self.yc, self.qc = self.ddstep(self.xc,
                                                    self.yc,
                                                    self.qc,
                                                    self.r,
                                                    self.L,
                                                    self.dt,
                                                    self.w1 * self.t * self.t,
                                                    self.w2 * self.t)
            self.t = self.t + self.dt
            self.x.append(self.xc)
            self.y.append(self.yc)

        if plot:
            plt.xlabel('x position')
            plt.ylabel('y position')
            plt.title('Problem 5a')
            plt.plot(self.x, self.y)
            self.plot.savefig('p6-5-a.pdf',
                              format='pdf',
                              dpi=1200)
            self.makeplot()

    def b(self, title, save_name, mu, sigma):
        runs = 100
        iterations = int(5.0 / self.dt + 1)
        x = []
        y = []
        for i in range(runs):
            self.reset()
            xerr = np.random.normal(mu, sigma, iterations)
            yerr = np.random.normal(mu, sigma, iterations)
            j = 0
            while (self.t < 5.0 - self.dt):
                self.xc, self.yc, self.qc = self.ddstep(self.xc + xerr[j],
                                                        self.yc + yerr[j],
                                                        self.qc,
                                                        self.r,
                                                        self.L,
                                                        self.dt,
                                                        self.w1 * self.t * self.t,
                                                        self.w2 * self.t)
                self.t = self.t + self.dt
                j += 1

            x.append(self.xc)
            y.append(self.yc)
        self.noise_x = x
        self.noise_y = y
        plt.xlabel('x position')
        plt.ylabel('y position')
        plt.title(title)
        plt.scatter(self.noise_x, self.noise_y)
        self.plot.savefig(save_name,
                          format='pdf',
                          dpi=1200)
        self.makeplot()

    def c(self, mu, sigma):
        self.reset()
        iterations = int(5.0 / self.dt + 1)
        xerr = np.random.normal(mu, sigma, iterations)
        yerr = np.random.normal(mu, sigma, iterations)
        x = []
        y = []
        i = 0
        while (self.t < 5.0 - self.dt):
            self.xc, self.yc, self.qc = self.ddstep(self.xc + xerr[i],
                                                    self.yc + yerr[i],
                                                    self.qc,
                                                    self.r,
                                                    self.L,
                                                    self.dt,
                                                    self.w1 * self.t * self.t,
                                                    self.w2 * self.t)
            self.t = self.t + self.dt
            x.append(self.xc)
            y.append(self.yc)
            i += 1

        self.a(plot=False)
        self.noise_x = x
        self.noise_y = y

        plt.xlabel('x position')
        plt.ylabel('y position')
        plt.title('Problem 5c')
        plt.plot(self.noise_x, self.noise_y, 'b-', self.x, self.y, 'r.')
        plt.xlim(-10, 90)
        plt.ylim(-20, 80)
        self.plot.savefig('p6-5-c.pdf',
                          format='pdf',
                          dpi=1200)
        self.makeplot()

    def d(self):
        #  assume final locations are in x & y
        mat = np.array([self.noise_x, self.noise_y])
        #  find covariance matrix
        cmat = np.cov(mat)
        # compute eigenvals and eigenvects of covariance
        eval, evec = linalg.eigh(cmat)
        #  find ellipse rotation angle
        angle = 180 * np.arctan2(evec[0, 1], evec[0, 0]) / np.pi
        # create ellipse
        ell = Ellipse((np.mean(self.noise_x), np.mean(self.noise_y)),
                      eval[0], eval[1], angle)
        #  make the ellipse subplot

        ax = self.plot.add_subplot(111, aspect='equal')
        ell.set_clip_box(ax.bbox)
        ell.set_alpha(0.1)
        ell.set_facecolor(np.random.rand(3))
        ax.add_artist(ell)
        # a = plt.subplot(111, aspect='equal')
        # ell.set_alpha(0.1)      # make the ellipse lighter
        # a.add_artist(ell)       # add this to the plot
        plt.plot(self.noise_x, self.noise_y)
        # plt.xlim(-10, 90)
        # plt.ylim(-20, 80)
        self.makeplot()

    def run(self):
        self.a()
        self.b('Problem 5b Default', 'p6-5-b-default.pdf', mu=0.0, sigma=0.3)
        self.b('Problem 5b Good', 'p6-5-b-good.pdf', mu=0.0, sigma=0.1)
        self.b('Problem 5b Great', 'p6-5-b-great .pdf', mu=0.0, sigma=0.01)
        self.c(0.0, 0.3)
        self.d()

    def makeplot(self):
        plt.show()
        plt.gcf().clear()


if __name__ == '__main__':
    p2 = diffdrive()
    p2.run()
    # p2.d()
