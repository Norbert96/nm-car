

from bezier import Bezier


def get_bezier_points(bezier, integer=True):
    ret = []
    for i in range(4):
        if integer:
            ret.append((int(bezier[i][0]), int(bezier[i][1])))
        else:
            ret.append((bezier[i][0], bezier[i][1]))
    return ret
