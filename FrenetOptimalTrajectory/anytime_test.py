#########################################
# Testing Scripts for Anytime Algorithm
# Please Follow README to install fot_planner package before running
#########################################
from fot_wrapper import _parse_hyperparameters, \
                    to_frenet_initial_conditions, query_anytime_planner_path
import fot_planner as fp
import numpy as np
import time

#########################################
# CONFIG and Set Up
#########################################
num_threads = 2
conds = {
    's0': 0,
    'target_speed': 20,
    'wp': [[0, 0], [50, 0], [150, 0]],
    'obs': [[48, -2, 52, 2], [98, -4, 102, 2], [98, 6, 102, 10],
            [128, 2, 132, 6]],
    'pos': [0, 0],
    'vel': [0, 0],
}  # paste output from debug log

initial_conditions = {
    'ps': conds['s0'],
    'target_speed': conds['target_speed'],
    'pos': np.array(conds['pos']),
    'vel': np.array(conds['vel']),
    'wp': np.array(conds['wp']),
    'obs': np.array(conds['obs'])
}

hyperparameters = {
    "max_speed": 25.0,
    "max_accel": 15.0,
    "max_curvature": 15.0,
    "max_road_width_l": 5.0,
    "max_road_width_r": 5.0,
    "d_road_w": 0.5,
    "dt": 0.2,
    "maxt": 5.0,
    "mint": 2.0,
    "d_t_s": 0.5,
    "n_s_sample": 2.0,
    "obstacle_clearance": 0.1,
    "kd": 1.0,
    "kv": 0.1,
    "ka": 0.1,
    "kj": 0.1,
    "kt": 0.1,
    "ko": 0.1,
    "klat": 1.0,
    "klon": 1.0,
    "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
}

# static elements of planner
wx = initial_conditions['wp'][:, 0]
wy = initial_conditions['wp'][:, 1]
obs = np.array(conds['obs'])

###############################
# Unit test on Anytime Planner
###############################
ic, misc = to_frenet_initial_conditions(initial_conditions)
hp = _parse_hyperparameters(hyperparameters)

# testing parameters
NUM_ITER = 10
TEST_INTERVAL_TIME = 0.002  # in seconds

# Calling Anytime Planner
planner = fp.FotPlanner(ic, hp)
planner.async_plan()

prev_cf, curr_cf = np.inf, np.inf
best_fot_rv_so_far = None

# Query anytime fot planner every TEST_INTERVAL_TIME,
# Check if cost of path is lower or at least the same for the next query
# This checks if implementation satisfies monotonicity of anytime algorithm,
# meaning that quality of result only gets better with time
for _ in range(NUM_ITER):
    result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, best_fot_rv_so_far = \
                query_anytime_planner_path(planner, return_rv_object=True)
    if (success):
        curr_cf = costs['cf']
        assert curr_cf <= prev_cf,\
            "cost must be non-increasing in anytime algo"
        print("Path Cost: ", curr_cf)
        prev_cf = curr_cf
    else:
        print("No Path Found")

    time.sleep(TEST_INTERVAL_TIME)

planner.stop_plan()
