import numpy as np
from typing import List, Tuple

def point_line_distance(point: Tuple[float, float], start: Tuple[float, float], end: Tuple[float, float]) -> float:
    if start == end:
        return np.linalg.norm(np.array(point) - np.array(start))
    n = abs((end[0] - start[0]) * (start[1] - point[1]) - (start[0] - point[0]) * (end[1] - start[1]))
    d = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    return n / d

def simplify_curve(points: List[Tuple[float, float]], epsilon: float) -> List[Tuple[float, float]]:
    if len(points) < 3:
        return points
    dmax = 0
    index = 0
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], points[0], points[-1])
        if d > dmax:
            index = i
            dmax = d
    if dmax > epsilon:
        results1 = simplify_curve(points[:index+1], epsilon)
        results2 = simplify_curve(points[index:], epsilon)
        return results1[:-1] + results2
    return [points[0], points[-1]]
