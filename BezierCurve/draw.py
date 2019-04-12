def get_bezier_points(bezier, scale):
    for i in range(scale):
        t = (i / scale) * 0.29352384841237594
        p = bezier.get(t)
        x.append(p[0])
        y.append(p[1])
    return x, y
