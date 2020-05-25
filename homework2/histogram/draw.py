import seaborn as sns
import matplotlib.pyplot as plt
from math import *

file = open("/home/vectorzhou/PublicCourse/homework2/histogram/pts.txt", "r")
n = int(file.readline())
pts = []

rmin, rmax = 1000000000, 0
zmin, zmax = 1000000000, 0

a = []
for i in range(n):
    x = float(file.readline())
    y = float(file.readline())
    z = float(file.readline())
    pts.append((x, y, z))
    r = sqrt(x * x + y * y + z * z)
    rmin = min(rmin, r)
    rmax = max(rmax, r)
    zmin = min(zmin, z)
    zmax = max(zmax, z)
    a.append(z)

sns.distplot(a, kde = False, axlabel = "height", label = "#points")
plt.show()
file.close()
