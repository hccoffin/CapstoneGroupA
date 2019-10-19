import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from queue import PriorityQueue
import time
from scipy.interpolate import RegularGridInterpolator

def generate_primitives_from_state(
    state, v_max, a_max, omega_max, alpha_max, prim_time,
    n_a=3, n_alpha=3, n_points=10,
):
    """Generates path primitives from a state"""
    x0, y0, theta0, v0, omega0 = state

    a = np.linspace(-a_max, a_max, n_a)
    alpha = np.linspace(-alpha_max, alpha_max, n_alpha)

    a_grid, alpha_grid = np.meshgrid(a, alpha, indexing='ij')

    t = np.linspace(0, prim_time, n_points)
    v = v0 + np.cumsum(a_grid[:, :, np.newaxis] * t[np.newaxis, np.newaxis, :], axis=2)
    v = np.clip(v, 0, v_max)
    omega = omega0 + np.cumsum(alpha_grid[:, :, np.newaxis] * t[np.newaxis, np.newaxis, :], axis=2)
    omega = np.clip(omega, -omega_max, omega_max)

    dt = t[1] - t[0]
    theta = theta0 + np.cumsum(dt * omega, axis=2)
    x = x0 + np.cumsum((dt * v) * np.cos(theta), axis=2)
    y = y0 + np.cumsum((dt * v) * np.sin(theta), axis=2)

    primitives = []
    for i in range(n_a):
        for j in range(n_alpha):
            primitives.append((
                x[i, j, :],
                y[i, j, :],
                theta[i, j, :],
                v[i, j, :],
                omega[i, j, :]
            ))

    return primitives

def eight_connected_distances(
    target_xy, x_min, x_max, y_min, y_max, obstacle_map
):
    """gets the distance from the target to each point by running Dijkstra's Algorithm on an 8-connected grid"""
    x_index = int(np.round(((target_xy[0] - x_min) / (x_max - x_min)) * (obstacle_map.shape[0] - 1)))
    y_index = int(np.round(((target_xy[1] - y_min) / (y_max - y_min)) * (obstacle_map.shape[1] - 1)))

    visited = set()
    priority_queue = PriorityQueue()
    priority_queue.put((0, (x_index, y_index)))
    distance_map = np.ones(obstacle_map.shape) * 1e9
    while not priority_queue.empty():
        to_expand = priority_queue.get()
        if to_expand[1] not in visited:
            x, y = to_expand[1]
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if (
                        (x + dx) >= 0 and (x + dx) < obstacle_map.shape[0] and
                        (y + dy) >= 0 and (y + dy) < obstacle_map.shape[0] and
                        obstacle_map[x + dx, y + dy] == 0
                    ):
                        dist_x = dx * (x_max - x_min) / obstacle_map.shape[0]
                        dist_y = dy * (y_max - y_min) / obstacle_map.shape[1]
                        new_dist = np.sqrt(dist_x ** 2 + dist_y ** 2) + to_expand[0]
                        priority_queue.put((new_dist, (x + dx, y + dy)))
            visited.add(to_expand[1])
            distance_map[to_expand[1][0], to_expand[1][1]] = to_expand[0]

    # dist_show = distance_map * (distance_map < 1e9)
    # plt.imshow(dist_show)
    # plt.colorbar()
    # plt.show()
    return distance_map

def heurisitic(state, v_max, a_max, dist_fn):
    """heurisitc function for weighted A* search"""
    time_to_reach = dist_fn(state[0:2]) / v_max
    return time_to_reach

def primitive_is_valid(primitive, x_min, x_max, y_min, y_max, obstacle_map):
    """checks that the path primitive stays in bounds and avoids obstacles"""
    if np.any(primitive[0] < x_min) or np.any(primitive[0] > x_max):
        return False
    elif np.any(primitive[1] < x_min) or np.any(primitive[1] > x_max):
        return False
    else:
        x_indices = np.round(((primitive[0] - x_min) / (x_max - x_min)) * (obstacle_map.shape[0] - 1))
        x_indices = x_indices.astype(np.int32)
        y_indices = np.round(((primitive[1] - y_min) / (y_max - y_min)) * (obstacle_map.shape[1] - 1))
        y_indices = y_indices.astype(np.int32)

        if np.any(obstacle_map[x_indices, y_indices] == 1):
            return False
        else:
            return True

def backtrack_to_get_path(final_state, state_info):
    """Backtracks along states to generate the full path from weights A*"""
    x_list = []
    y_list = []
    theta_list = []
    v_list = []
    omega_list = []
    state = final_state[1]
    while state_info[state]['prev_state'] is not None:
        primitive = state_info[state]['primitive']
        x_list.append(primitive[0])
        y_list.append(primitive[1])
        theta_list.append(primitive[2])
        v_list.append(primitive[3])
        omega_list.append(primitive[4])
        state = state_info[state]['prev_state']

    x_list.reverse()
    y_list.reverse()
    theta_list.reverse()
    v_list.reverse()
    omega_list.reverse()

    path = (
        np.concatenate(x_list),
        np.concatenate(y_list),
        np.concatenate(theta_list),
        np.concatenate(v_list),
        np.concatenate(omega_list)
    )

    return np.stack(path, axis=0)

def weighted_A_star(
    init_state, target_xy,
    v_max, a_max, omega_max, alpha_max, prim_time,
    x_min, x_max, y_min, y_max, obstacle_map, tolerance, 
    weight=1, n_a=3, n_alpha=3, n_points=10,
):
    """runs weighted A* from initial state to target

    Finds a path (a sequence of states (x, y, theta, v, omega)) which goes from
    the initial state to within a distance 'tolerance' of the target (x, y)
    position. 

    Args:
        init_state (iterable): initial state (x, y, theta, v, omega)
        target_xy (iterable): target position (x, y)
        v_max (float): maximum velocity of robot
        a_max (float): maximum acceleration of robot
        omega_max (float): maximum angular velocity of robot
        alpha_max (float): maximum angular acceleration of robot
        prim_time (float): duration of one path primitive
        x_min (float): min x value of obstacle map
        x_max (float): max x value of obstacle map
        y_min (float): min y value of obstacle map
        y_max (float): max y value of obstacle map
        obstacle_map (np.ndarray): 2 dimensional of size (n_x, n_y) which is
            0 in obstacle free positions and 1 where there are obstacles
        tolerance (float): the search will stop when the path is within
            tolerance distance from target_xy
        weight (float): weight for weighted A*. Defaults to 1 (regular A*)
        n_a (int): number of acceleration values for primitives. At each state
            which is expanded, n_a * n_alpha primitives will be generated. 
            Defaults to 3
        n_alpha (int): number of angular acceleration values for primitives. 
            At each state which is expanded, n_a * n_alpha primitives will be 
            generated. Defaults to 3
        n_points (int): number of points to be checked along each primitive to
            check if it is valid. Defaults to 10

    Returns:
        np.ndarray: 5 by (n_points * number of primitives in path) array of 
            states for the path found from weighted A*
    """
    distance_map = eight_connected_distances(
        target_xy, x_min, x_max, y_min, y_max, obstacle_map
    )
    xs = np.linspace(x_min, x_max, distance_map.shape[0])
    ys = np.linspace(y_min, y_max, distance_map.shape[1])
    # create an interpolator for the distance (found that using just nearest 
    # neighbor does not work well when the angular velocities are small)
    dist_fn = RegularGridInterpolator((xs, ys), distance_map, method='linear')

    state_info = dict()
    state_info[init_state] = {
        'prev_state': None,
        'primitive': None,
        'cur_t': 0
    }
    pq = PriorityQueue()

    num_expanded_state = 0
    priority_state = (
        weight * heurisitic(init_state, v_max, a_max, dist_fn), 
        init_state
    )
    while dist_fn(priority_state[1][0:2]) > tolerance or priority_state[1][3] != 0:
        cur_t = state_info[priority_state[1]]['cur_t'] + prim_time
        primitives = generate_primitives_from_state(
            priority_state[1], v_max, a_max, omega_max, alpha_max, prim_time,
            n_a=n_a, n_alpha=n_alpha, n_points=n_points
        )
        for p in primitives:
            if primitive_is_valid(p, x_min, x_max, y_min, y_max, obstacle_map):
                end_state = end_state = tuple([s[-1] for s in p])
                if end_state not in state_info:
                    state_info[end_state] = {
                        'prev_state': priority_state[1],
                        'primitive': p,
                        'cur_t': cur_t
                    }
                    pq.put((
                        weight * heurisitic(end_state, v_max, a_max, dist_fn) + cur_t, 
                        end_state
                    ))
        priority_state = pq.get()
        num_expanded_state += 1

    return backtrack_to_get_path(priority_state, state_info)

if __name__ == '__main__':
    """an example of the planner being used (color of path is the velocity)"""
    v_max = .1
    a_max = .1 / 50
    omega_max = np.pi / 5
    alpha_max = np.pi / 20
    n_a = 5
    n_alpha = 5
    n_points = 30
    prim_time = 1

    init_state = (.01, .01, 0, 0, 0)

    x_min, x_max, y_min, y_max = 0, 1, 0, 1
    obstacle_map = np.zeros((100, 100))
    obstacle_map[20:30, 0:80] = 1
    obstacle_map[35:70, 20:100] = 1
    tolerance = .05
    weight = 100

    target_xy = [.8, .8]

    path = weighted_A_star(
        init_state, target_xy,
        v_max, a_max, omega_max, alpha_max, prim_time,
        x_min, x_max, y_min, y_max, obstacle_map, tolerance, 
        weight=weight, n_a=n_a, n_alpha=n_alpha, n_points=n_points
    )

    print(path.shape)

    segments = [np.stack([path[0, i:i+2], path[1, i:i+2]], axis=1) for i in range(path[0].shape[0] - 1)]
    cmap = plt.get_cmap('coolwarm')
    lines = LineCollection(segments, linewidths=1, colors=cmap(plt.Normalize()(path[3][1:])))

    fig, ax = plt.subplots()
    ax.pcolormesh(np.linspace(0, 1, 100), np.linspace(0, 1, 100), obstacle_map.T)
    ax.add_collection(lines)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.show()