import numpy as np
import matplotlib.pyplot as plt

# 0.6 0.5 0.4 -0.6 -0.5 -0.4 -0.3 -0.2    0      0.3  0.2
# 103 108 141 -125 -140 -174 -239 -328 9999999.0 197 244 
L = 0.213
#R = np.array([1.03/2, 1.08/2, 1.41/2, 1.97/2, 2.44/2, 999999.0, -3.28/2, -2.39/2, -1.74/2, -1.40/2, -1.25/2])
R = np.array([0.527, 0.639, 0.761, 1.04, 1.573, 2.723, 999999.0, -8.154, -2.572, -1.582, -1.062, -0.802, -0.691])
y = np.arctan(L/R)
print(y)
x = np.array([0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6])
#print(y)
p = np.polyfit(x, y, 3)
#np.savetxt('output.txt', p, fmt='%f', delimiter="")
print(p)
x_test = []
y_test = []
for i in np.arange(-0.6, 0.6, 0.05):
    x_test.append(i)
    y_test.append(np.polyval(p, i))
plt.plot(x_test, y_test, 'r')
plt.plot(x, y, 'b')
plt.show()
print(np.polyfit(np.array(y_test), np.array(x_test), 3))
