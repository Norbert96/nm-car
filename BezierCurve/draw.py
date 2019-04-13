def get_bezier_points(bezier, scale):
    x = []
    y = []
    for i in range(scale+1):
        t = (i / scale)
        p = bezier.get(t)
        x.append(p[0])
        y.append(p[1])
    return x, y
