from bezier import Bezier
from draw import * 

import numpy as np
import matplotlib.pyplot as plt



x = np.array([100, 150+50, 110, 150])
y = np.array([25, 50+25, 100, 195])
x1 = np.array([100, 50, 35, 30])
y1 = np.array([25, 0, 35, 30])

b = Bezier(x, y)
b1 = Bezier(x1, y1)


# b.printS_nodes()



# bs = b.reduce()

# for i in bs:
# 	b.print_nodes()


x, y = get_bezier_points(b, 20)
x1, y1 = get_bezier_points(b1, 20)


# print(len(bs))
# for i in bs:
# 	plt.plot(i.x, i.y, 'x')
# 	print("t1: {} t2: {}".format(i.t1, i.t2))
plt.plot(b.x, b.y, 'o')
plt.plot(b1.x, b1.y, 'o')
plt.plot(x,y)
plt.plot(x1,y1)


# plt.plot(x, y, 'bo')
# plt.legend(['Linear', 'Cubic Spline', 'True'])
plt.axis([0, 200, 0, 200])
plt.show()


