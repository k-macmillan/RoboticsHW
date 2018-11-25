import numpy as np


def problem17_3():
    # Specific functions to the problem
    def updateXhat0(x, y, t):
        return np.array([x + 0.1 * y, y - 0.1 * np.cos(x) + 0.5 * np.sin(t)])

    def updateF(x):
        return np.array([[1.0, 0.1],
                        [0.1 * np.sin(x), 1.0]])
    # Time information
    dt = 0.1
    t = np.arange(0, 2 * np.pi + dt, dt)  # 64 points

    # Starting data
    V = np.array([[0.2, 0.0],
                 [0.0, 0.2]])
    W = 0.25
    H = np.array([1, 0])

    x0 = np.array(0, 0)

    P = np.array([[0.0, 0.0],
                  [0.0, 0.0]])

    z = np.array([1.15, 0.5])

    # Predict State
    x_hat0 = updateXhat0(x0[0], x0[1], 0)

    # Predict estimate covariance
    P0 = updateP0(V, updateF(x0[0]), P)

    # Optimal Kalman gain
    K = updateK(H, P0, W)

    # Update state estimate
    x_hat1 = updateXhat1(x_hat0, K, z)

    # Update estimate covariance
    P1 = updateP1(np.identity(2), K, H, P0)

    print()
    print('x_hat0: {}'.format(x_hat0))
    print('P =     {}'.format(P0[0]))
    print('        {}'.format(P0[1]))
    print('K =     {}'.format(K[0]))
    print('        {}'.format(K[1]))
    print('x_hat1: {}'.format(x_hat1))
    print('P =     {}'.format(P1[0]))
    print('        {}'.format(P1[1]))
    print()


def sanityCheck():
    # Specific functions to the problem
    def updateXhat0(x, y, t):
        return np.array([x + 0.1 * y, y - 0.1 * np.cos(x) + 0.04 * np.sin(t)])

    def updateF(x):
        return np.array([[1.0, 0.1],
                        [0.1 * np.sin(x), 1.0]])

    V = np.array([[0.1, 0.01],
                  [0.01, 0.1]])
    W = np.array([[0.05, 0.0],
                  [0.0, 0.05]])

    H = np.array([[1, 0],
                 [0, 1]])

    x0 = np.array([1, 1])

    P0 = np.array([[0.5, 0.0],
                   [0.0, 0.5]])

    z = np.array([1.15, 0.5])

    # Predict State
    x_hat0 = updateXhat0(x0[0], x0[1], 0)

    # Predict estimate covariance
    P0 = updateP0(V, updateF(x0[0]), P0)

    # Optimal Kalman gain
    K = updateK(H, P0, W)

    # Update state estimate
    fake_x_hat = np.array([1.1, 0.45969769])
    x_hat1 = updateXhat1(fake_x_hat, K, z)

    # Update estimate covariance
    P1 = updateP1(np.identity(2), K, H, P0)

    print()
    print('x_hat0: {}'.format(x_hat0))
    print('P =     {}'.format(P0[0]))
    print('        {}'.format(P0[1]))
    print('K =     {}'.format(K[0]))
    print('        {}'.format(K[1]))
    print('x_hat1: {}'.format(x_hat1))
    print('P =     {}'.format(P1[0]))
    print('        {}'.format(P1[1]))
    print()


# Common equations
def updateXhat1(x_hat, K, z):
    return x_hat + np.matmul(K, z - x_hat)


def updateP0(V, F, P):
    return np.matmul(np.matmul(F, P), F.T) + V


def updateK(H, P, W):
    preK = np.linalg.inv(np.matmul(np.matmul(H, P), H.T) + W)
    return np.matmul(np.matmul(P, H.T), preK)


def updateP1(I, K, H, P0):
    return np.matmul(I - np.matmul(K, H), P0)


if __name__ == '__main__':
    # updateF(1)
    sanityCheck()
