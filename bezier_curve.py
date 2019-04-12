import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


# x = np.array([0.5, 1, 1.5, 1, 2, 3, 4, 1, 0.5])
# y = np.array([1, 2, 3, 3.2, 4, 3, 2.5, 2, 1])
# # x = np.arange(0, 2 * np.pi + np.pi / 4, 2 * np.pi / 8)

# print(len(x))
# # y = np.abs(-x + 1)
# tck = interpolate.splrep(x, y, s=0)
# xnew = np.arange(0, 2 * np.pi, np.pi / 50)
# ynew = interpolate.splev(xnew, tck, der=0)


# plt.figure()
# plt.plot(x, y, 'x', xnew, ynew, xnew, np.sin(xnew), x, y, 'b')
# plt.legend(['Linear', 'Cubic Spline', 'True'])
# plt.axis([-0.05, 6.33, -1.05, 6.05])
# plt.title('Cubic-spline interpolation')
# plt.show()

x = np.array([15, 35, 50, 70, 80, 80, 60, 45, 40, 30, 15, 10, 15])
y = np.array([15, 10, 25, 35, 40, 70, 75, 80, 60, 50, 50, 30, 15])
# x = np.arange(0, 101)
# y = np.arange(0, 101) * 2
# y = np.abs(y)
tck, u = interpolate.splprep([x, y], s=0)
unew = np.arange(0, 1.01, 0.01)
out = interpolate.splev(unew, tck, der=0)
der = interpolate.splev(unew, tck, der=1)
print(np.array(der))
der = np.array(der)
out = np.array(out)
grad = der / np.sqrt((np.square(der[1]) + np.square(der[0])))
print('grad {}'.format(grad))
print(((grad[0] / grad[1])))
sign = np.zeros(grad[0].size)
print(sign)
sign = (((grad[1] / grad[0]) > 0) == (grad[0] > 0)) * 2 - 1
print(sign)
n = out + (np.array([grad[1], grad[0]]) * 5)


print(out)
plt.figure()
print('derivative')
print(der)

plt.plot(out[0], out[1], n[0], n[1])
plt.legend(['Linear', 'Cubic Spline', 'True'])
plt.axis([0, 100, 0, 100])
plt.title('Spline of parametrically-defined curve')
plt.show()
