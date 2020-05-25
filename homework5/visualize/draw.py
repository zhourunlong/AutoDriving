import matplotlib.pyplot as plt
from math import *

plt.figure(figsize=(10,10))

file = open("/home/vectorzhou/AutoDriving/homework5/visualize/pts.txt", "r")
n = int(file.readline())
x = []
y = []
for i in range(n - 300):
    x.append(float(file.readline()))
    y.append(float(file.readline()))

plt.scatter(x, y, c = 'red', marker = 'o', label = 'input');

ext_x = []
ext_y = []
for i in range(300):
    ext_x.append(float(file.readline()))
    ext_y.append(float(file.readline()))

plt.scatter(ext_x, ext_y, s = 20, c = 'lightcoral', marker = '.', label = 'interpolated');

plt.legend()
plt.show()
file.close()
