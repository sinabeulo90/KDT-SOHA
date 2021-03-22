import numpy as np

def get_linear_function(points, loss="cauchy"):
    xs, ys = [], []

    for x, y in points:
        xs.append(x)
        ys.append(y)
    
    if len(xs) > 15 and len(ys) > 15:
        xs = np.array(xs)
        ys = np.array(ys)

        y0 = np.polyfit(ys, xs, 2)
        result_f = np.poly1d(y0)
        return result_f

    return None