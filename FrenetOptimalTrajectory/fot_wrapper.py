import numpy as np
import os

from ctypes import c_double, c_int, POINTER, Structure, CDLL, byref

try:
    from py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters, \
        FrenetReturnValues, MAX_PATH_LENGTH
except:
    from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory\
        .py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters, \
         FrenetReturnValues, MAX_PATH_LENGTH

try:
    cdll = CDLL("build/libFrenetOptimalTrajectory.so")
except:
    cdll = CDLL("{}/dependencies/frenet_optimal_trajectory_planner/"
                "build/libFrenetOptimalTrajectory.so".format(
                    os.getenv("PYLOT_HOME")))

_c_double_p = POINTER(c_double)

# func / return type declarations for C++ run_fot
_run_fot = cdll.run_fot
_run_fot.argtypes = (
    POINTER(FrenetInitialConditions),
    POINTER(FrenetHyperparameters),
    POINTER(FrenetReturnValues),
)
_run_fot.restype = None

# func / return type declarations for C++ to_frenet_initial_conditions
_to_frenet_initial_conditions = cdll.to_frenet_initial_conditions
_to_frenet_initial_conditions.restype = None
_to_frenet_initial_conditions.argtypes = (c_double, c_double, c_double,
                                          c_double, c_double, c_double,
                                          _c_double_p, _c_double_p, c_int,
                                          _c_double_p)


def _parse_hyperparameters(hp):
    return FrenetHyperparameters(
        hp["max_speed"], hp["max_accel"], hp["max_curvature"],
        hp["max_road_width_l"], hp["max_road_width_r"], hp["d_road_w"],
        hp["dt"], hp["maxt"], hp["mint"], hp["d_t_s"], hp["n_s_sample"],
        hp["obstacle_clearance"], hp["kd"], hp["kv"], hp["ka"], hp["kj"],
        hp["kt"], hp["ko"], hp["klat"], hp["klon"], hp["num_threads"])


def run_fot(initial_conditions, hyperparameters):
    """ Return the frenet optimal trajectory given initial conditions in
    cartesian space.

    Args:
        initial_conditions (dict): dict containing the following items
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]

        hyperparameters (dict): a dict of optional hyperparameters
            max_speed (float): maximum speed [m/s]
            max_accel (float): maximum acceleration [m/s^2]
            max_curvature (float): maximum curvature [1/m]
            max_road_width_l (float): maximum road width to the left [m]
            max_road_width_r (float): maximum road width to the right [m]
            d_road_w (float): road width sampling discretization [m]
            dt (float): time sampling discretization [s]
            maxt (float): max prediction horizon [s]
            mint (float): min prediction horizon [s]
            d_t_s (float): target speed sampling discretization [m/s]
            n_s_sample (float): sampling number of target speed
            obstacle_clearance (float): obstacle radius [m]
            kd (float): positional deviation cost
            kv (float): velocity cost
            ka (float): acceleration cost
            kj (float): jerk cost
            kt (float): time cost
            ko (float): dist to obstacle cost
            klat (float): lateral cost
            klon (float): longitudinal cost

    Returns:
        result_x (np.ndarray(float)): x positions of fot, if it exists
        result_y (np.ndarray(float)): y positions of fot, if it exists
        speeds (np.ndarray(float)): speeds of fot, if it exists
        ix (np.ndarray(float)): spline x of fot, if it exists
        iy (np.ndarray(float)): spline y of fot, if it exists
        iyaw (np.ndarray(float)): spline yaws of fot, if it exists
        d (np.ndarray(float)): lateral offset of fot, if it exists
        s (np.ndarray(float)): longitudinal offset of fot, if it exists
        speeds_x (np.ndarray(float)): x speeds of fot, if it exists
        speeds_y (np.ndarray(float)): y speeds of fot, if it exists
        params (dict): next frenet coordinates, if they exist
        costs (dict): costs of best frenet path, if it exists
        success (bool): whether a fot was found or not
    """
    # parse initial conditions and convert to frenet coordinates
    fot_initial_conditions, misc = to_frenet_initial_conditions(
        initial_conditions)

    # parse hyper parameters
    fot_hp = _parse_hyperparameters(hyperparameters)

    # initialize return values
    fot_rv = FrenetReturnValues(0)

    # run the planner
    _run_fot(fot_initial_conditions, fot_hp, fot_rv)

    x_path = np.array([fot_rv.x_path[i] for i in range(MAX_PATH_LENGTH)])
    y_path = np.array([fot_rv.y_path[i] for i in range(MAX_PATH_LENGTH)])
    speeds = np.array([fot_rv.speeds[i] for i in range(MAX_PATH_LENGTH)])
    ix = np.array([fot_rv.ix[i] for i in range(MAX_PATH_LENGTH)])
    iy = np.array([fot_rv.iy[i] for i in range(MAX_PATH_LENGTH)])
    iyaw = np.array([fot_rv.iyaw[i] for i in range(MAX_PATH_LENGTH)])
    d = np.array([fot_rv.d[i] for i in range(MAX_PATH_LENGTH)])
    s = np.array([fot_rv.s[i] for i in range(MAX_PATH_LENGTH)])
    speeds_x = np.array([fot_rv.speeds_x[i] for i in range(MAX_PATH_LENGTH)])
    speeds_y = np.array([fot_rv.speeds_y[i] for i in range(MAX_PATH_LENGTH)])
    params = {
        "s": fot_rv.params[0],
        "s_d": fot_rv.params[1],
        "d": fot_rv.params[2],
        "d_d": fot_rv.params[3],
        "d_dd": fot_rv.params[4],
    }
    costs = {
        "c_lateral_deviation": fot_rv.costs[0],
        "c_lateral_velocity": fot_rv.costs[1],
        "c_lateral_acceleration": fot_rv.costs[2],
        "c_lateral_jerk": fot_rv.costs[3],
        "c_lateral": fot_rv.costs[4],
        "c_longitudinal_acceleration": fot_rv.costs[5],
        "c_longitudinal_jerk": fot_rv.costs[6],
        "c_time_taken": fot_rv.costs[7],
        "c_end_speed_deviation": fot_rv.costs[8],
        "c_longitudinal": fot_rv.costs[9],
        "c_inv_dist_to_obstacles": fot_rv.costs[10],
        "cf": fot_rv.costs[11],
    }

    success = fot_rv.success

    # remove values after last calculated waypoint
    ind = -1
    if success:
        ind = np.where(np.isnan(x_path))[0][0]

    return x_path[:ind], y_path[:ind], speeds[:ind], \
           ix[:ind], iy[:ind], iyaw[:ind], d[:ind], s[:ind], \
           speeds_x[:ind], speeds_y[:ind], params, costs, success


def to_frenet_initial_conditions(initial_conditions):
    """ Convert the cartesian initial conditions into frenet initial conditions.

    Args:
        initial_conditions (dict): dictionary containing
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]
    Returns:
        FrenetInitialConditions, dictionary for debugging
    """
    # parse the initial conditions
    ps = initial_conditions['ps']
    pos = initial_conditions['pos']
    vel = initial_conditions['vel']
    wp = initial_conditions['wp']
    obs = initial_conditions['obs']
    target_speed = initial_conditions['target_speed']
    if obs.shape[0] == 0:
        obs = np.empty((0, 4))
    x = pos[0].item()
    y = pos[1].item()
    vx = vel[0].item()
    vy = vel[1].item()
    wx = wp[:, 0].astype(np.float64)
    wy = wp[:, 1].astype(np.float64)
    o_llx = np.copy(obs[:, 0]).astype(np.float64)
    o_lly = np.copy(obs[:, 1]).astype(np.float64)
    o_urx = np.copy(obs[:, 2]).astype(np.float64)
    o_ury = np.copy(obs[:, 3]).astype(np.float64)
    forward_speed = np.hypot(vx, vy).item()

    # construct return array and convert initial conditions
    misc = np.zeros(5)
    _to_frenet_initial_conditions(c_double(ps), c_double(x), c_double(y),
                                  c_double(vx), c_double(vy),
                                  c_double(forward_speed),
                                  wx.ctypes.data_as(_c_double_p),
                                  wy.ctypes.data_as(_c_double_p),
                                  c_int(len(wx)),
                                  misc.ctypes.data_as(_c_double_p))

    # return the FrenetInitialConditions structure
    return FrenetInitialConditions(
        misc[0],  # c_s
        misc[1],  # c_speed
        misc[2],  # c_d
        misc[3],  # c_d_d
        misc[4],  # c_d_dd
        target_speed,  # target speed
        wx.ctypes.data_as(_c_double_p),  # waypoints x position
        wy.ctypes.data_as(_c_double_p),  # waypoints y position
        len(wx),
        o_llx.ctypes.data_as(_c_double_p),  # obstacles lower left x
        o_lly.ctypes.data_as(_c_double_p),  # obstacles lower left y
        o_urx.ctypes.data_as(_c_double_p),  # obstacles upper right x
        o_ury.ctypes.data_as(_c_double_p),  # obstacles upper right y
        len(o_llx),
    ), misc


#############################################################
# For Anytime Planner Implementation
#############################################################
def query_anytime_planner_path(fot_planner, return_rv_object=False):
    # return value should be separate for each path
    fot_rv = FrenetReturnValues(0)
    fot_planner.get_path(fot_rv)

    x_path = np.array([fot_rv.x_path[i] for i in range(MAX_PATH_LENGTH)])
    y_path = np.array([fot_rv.y_path[i] for i in range(MAX_PATH_LENGTH)])
    speeds = np.array([fot_rv.speeds[i] for i in range(MAX_PATH_LENGTH)])
    ix = np.array([fot_rv.ix[i] for i in range(MAX_PATH_LENGTH)])
    iy = np.array([fot_rv.iy[i] for i in range(MAX_PATH_LENGTH)])
    iyaw = np.array([fot_rv.iyaw[i] for i in range(MAX_PATH_LENGTH)])
    d = np.array([fot_rv.d[i] for i in range(MAX_PATH_LENGTH)])
    s = np.array([fot_rv.s[i] for i in range(MAX_PATH_LENGTH)])
    speeds_x = np.array([fot_rv.speeds_x[i] for i in range(MAX_PATH_LENGTH)])
    speeds_y = np.array([fot_rv.speeds_y[i] for i in range(MAX_PATH_LENGTH)])
    params = {
        "s": fot_rv.params[0],
        "s_d": fot_rv.params[1],
        "d": fot_rv.params[2],
        "d_d": fot_rv.params[3],
        "d_dd": fot_rv.params[4],
    }
    costs = {
        "c_lateral_deviation": fot_rv.costs[0],
        "c_lateral_velocity": fot_rv.costs[1],
        "c_lateral_acceleration": fot_rv.costs[2],
        "c_lateral_jerk": fot_rv.costs[3],
        "c_lateral": fot_rv.costs[4],
        "c_longitudinal_acceleration": fot_rv.costs[5],
        "c_longitudinal_jerk": fot_rv.costs[6],
        "c_time_taken": fot_rv.costs[7],
        "c_end_speed_deviation": fot_rv.costs[8],
        "c_longitudinal": fot_rv.costs[9],
        "c_inv_dist_to_obstacles": fot_rv.costs[10],
        "cf": fot_rv.costs[11],
    }

    success = fot_rv.success
    # remove values after last calculated waypoint
    ind = -1
    if success:
        ind = np.where(np.isnan(x_path))[0][0]

    if return_rv_object:
        return x_path[:ind], y_path[:ind], speeds[:ind], \
            ix[:ind], iy[:ind], iyaw[:ind], d[:ind], s[:ind], \
            speeds_x[:ind], speeds_y[:ind], params, costs, success, \
            fot_rv
    else:
        return x_path[:ind], y_path[:ind], speeds[:ind], \
            ix[:ind], iy[:ind], iyaw[:ind], d[:ind], s[:ind], \
            speeds_x[:ind], speeds_y[:ind], params, costs, success
