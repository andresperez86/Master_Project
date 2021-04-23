import numpy as np
from .astar import astar_path, astar_search

def bcd_preprocess(area_map):
    """
    Returns matrix that in the form
    that is read by the main algorithm.
    """
    matrix = np.full(area_map.shape, 0, dtype=np.uint8)
    matrix[area_map == 0] = 255
    return matrix

def is_bounded(coord, shape):
    """
    Checks if a coord (x,y) is within bounds.
    """
    x, y = coord
    g, h = shape
    lesser = x < 0 or y < 0
    greater = x >= g or y >= h
    if lesser or greater:
        return False
    return True

def is_valid(coord, area_map, obstacle=-1):
    """
    Check is a coord (x,y) is bounded and not
    on an obstacle.
    """
    coord = tuple(coord)
    is_b = is_bounded(coord, area_map.shape)

    if is_b:
        is_on_obs = False
        if isinstance(obstacle, list):
            for obs in obstacle:
                is_on_obs |= area_map[coord] == obs
        else:
            is_on_obs = area_map[coord] == obstacle
        if not is_on_obs:
            return True
    return False

def is_valid_vectorized(coords, matrix):
    assert coords.shape[1] == 2
    h, w = matrix.shape
    x, y = coords.T
    is_within_bounds = (x >= 0) & (x < h) & (y >= 0) & (y < w)
    x = np.clip(x.copy(), 0, h - 1)
    y = np.clip(y.copy(), 0, w - 1)
    is_not_on_obstacle = (matrix[x, y] != 0) & (matrix[x, y] != 150)
    return is_within_bounds & is_not_on_obstacle


def backtracking_list(memory, _, matrix, x, y):
    bt_cond_points = {
        "r": lambda p: p + np.array([[0, 1]]),  # right
        "tr": lambda p: p + np.array([[-1, 1]]),  # top-right
        "t": lambda p: p + np.array([[-1, 0]]),  # top
        "tl": lambda p: p + np.array([[-1, -1]]),  # top-left
        "l": lambda p: p + np.array([[0, -1]]),  # left
        "bl": lambda p: p + np.array([[1, -1]]),  # bottom-left
        "b": lambda p: p + np.array([[1, 0]]),  # bottom
        "br": lambda p: p + np.array([[1, 1]]),  # bottom-right
    }
    memory_ = np.array(memory)
    assert memory_.shape[1] == 2, "you've messed up something"

    eight_di = {k: bt_cond_points[k](memory_) for k in bt_cond_points}
    is_valid_eight = {k: is_valid_vectorized(eight_di[k], matrix) for k in eight_di}
    cond_a = np.int0(is_valid_eight["r"] & ~is_valid_eight["br"])
    cond_b = np.int0(is_valid_eight["r"] & ~is_valid_eight["tr"])
    cond_c = np.int0(is_valid_eight["l"] & ~is_valid_eight["bl"])
    cond_d = np.int0(is_valid_eight["l"] & ~is_valid_eight["tl"])
    cond_e = np.int0(is_valid_eight["b"] & ~is_valid_eight["bl"])
    cond_f = np.int0(is_valid_eight["b"] & ~is_valid_eight["br"])
    μ_of_s = (cond_a + cond_b + cond_c + cond_d + cond_e + cond_f)

    backtrack_points = memory_[μ_of_s > 0]
    if backtrack_points.shape[0] == 0:
        backtrack_points = memory_[is_valid_eight["r"] | is_valid_eight["l"] |
                                   is_valid_eight["t"] | is_valid_eight["b"]]

    if backtrack_points.shape[0] == 0:
        return (0, 0), True
    else:
        closest_point_idx = ((backtrack_points - np.array([x, y])) ** 2).sum(axis=1).argmin()
        return tuple(backtrack_points[closest_point_idx]), False


def visit(matrix, x, y, memory):
    matrix[(x, y)] = 150  # 150 == visited
    memory.append((x, y))
    return x, y


def move_like_oxen(matrix, diameter, x, y, memory):
    # TODO :: Variable diameter support
    udlr = {
        "u": lambda x, y: (x - diameter, y),
        "d": lambda x, y: (x + diameter, y),
        "l": lambda x, y: (x, y - diameter),
        "r": lambda x, y: (x, y + diameter)
    }
    u = "u"
    d = "d"
    r = "r"
    l = "l"
    visit(matrix, x, y, memory)

    while True:
        dir_ = [u, d, r, l]
        while len(dir_) > 0:
            d_ = dir_.pop(0)
            x_, y_ = udlr[d_](x, y)
            if is_valid((x_, y_), matrix, [0, 150]):
                x, y = visit(matrix, x_, y_, memory)
                break
            elif d_ == l:
                return x, y



def bcd_main(matrix, start_point):
    width, height = matrix.shape
    # May add support for variable radius.
    radius = 0.5
    diameter = int(2*radius)
    x, y = start_point
    memory = []
    backtrack_counts = 0
    point_find_failed = 0
    while True:
        critical_x, critical_y = move_like_oxen(matrix, diameter, x, y, memory)
        next_, is_end = backtracking_list(
            memory, diameter, matrix, critical_x, critical_y)
        x, y = next_
        if is_end:
            break
        else:
            start = (critical_x, critical_y)
            end = (x, y)
            path = astar_search(start, end, matrix, diameter, width, height)
            astar_path(matrix, memory, path)
    return memory

def bcd(area_map, start_point):
    """
    Runs the boustrophedon cell decomposition algorithm
    with astar for point search and returns th generated path.

    PARAMETERS
    ---
    area_map : Area map to be covered, 2-dim numpy array.
        coverage region =  0
        obstacle region = -1

    start_point : Drone deployment point on the `area_map`, tuple (x,y)

    RETURNS
    ---
    coverage_path : main coverage path.
    """
    matrix = bcd_preprocess(area_map)
    coverage_path = bcd_main(matrix, start_point)
    return coverage_path
