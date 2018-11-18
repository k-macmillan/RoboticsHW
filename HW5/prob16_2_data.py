import numpy as np


dist_sens = np.array([[2.2411549, 1.8286673, 2.3295015],
                     [2.3366108, 1.9243295, 2.4867167],
                     [1.9687234, 1.8972737, 2.5489412],
                     [2.1240351, 2.0961534, 1.9834876],
                     [2.3984044, 1.7985819, 2.6153805],
                     [2.3523899, 1.8377782, 1.8132444],
                     [2.1074266, 1.8358201, 2.5563951],
                     [2.4711542, 1.8875839, 2.2031291],
                     [2.1998000, 1.8116113, 2.3117542],
                     [2.2710086, 1.8701890, 2.3495262],
                     [2.3530473, 1.7646824, 2.0109293],
                     [2.4391559, 1.9499153, 2.5030771],
                     [2.2066306, 1.9243432, 2.3561112],
                     [2.3000099, 1.8309696, 2.3097754],
                     [2.2235766, 1.8453219, 2.2940692],
                     [2.1396901, 1.8390955, 2.1904604],
                     [2.0929719, 1.7978329, 2.5693897],
                     [2.3154159, 1.8217245, 1.9332188],
                     [2.3716302, 1.9558670, 2.3002433],
                     [2.2611420, 1.8654487, 2.5508342],
                     [2.1415088, 1.7836290, 2.6884786],
                     [2.2088487, 1.9245743, 2.5037028],
                     [2.2714614, 1.8918415, 2.7112663],
                     [2.3345816, 1.8275421, 2.1656644],
                     [2.3052296, 1.8494488, 2.1940472],
                     [2.1600041, 1.7632971, 2.2703708],
                     [2.0630943, 1.8396972, 2.6488544],
                     [2.0997821, 1.8412331, 2.1828831],
                     [2.3037175, 1.7761007, 2.2959535],
                     [2.4536524, 1.8542271, 2.0446945],
                     [2.3909478, 1.8649815, 2.7852822],
                     [2.1195966, 1.9533324, 2.5700007],
                     [2.0205112, 1.8857815, 2.1113650],
                     [2.1708006, 1.7115595, 2.1215336],
                     [2.0800748, 1.9403332, 2.3126032],
                     [2.3332722, 1.8530670, 2.4687277],
                     [2.0826115, 1.8279041, 2.6104026],
                     [2.2652480, 1.9058054, 2.3165716],
                     [2.3734464, 1.9632258, 2.0907554],
                     [2.0563260, 1.9367908, 2.2130578]])

mean = np.mean(dist_sens, dtype=np.float64, axis=0)
std = np.std(dist_sens, dtype=np.float64, axis=0)

zdist_sens = dist_sens - mean
zmean = np.mean(zdist_sens)
zstd = std / np.std(zdist_sens, dtype=np.float64, axis=0)

print(mean)
print(std)
print(zmean)
print(zstd)
