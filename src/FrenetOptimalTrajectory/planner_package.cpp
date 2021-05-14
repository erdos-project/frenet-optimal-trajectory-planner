#include "planner_package.h"
#include "py_cpp_struct.h"
#include <structmember.h>

/**
 * FotPlanner Defintions
 */
PyObject *FotPlanner_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    FotPlanner *self = (FotPlanner *)type->tp_alloc(type, 1);
    self->fot = NULL;
    return (PyObject *)self;
}

/**
 * Initializer for FotPlanner, set up Anytime Planner
 * with initial condition and hyperparameters
 */
int FotPlanner_init(PyObject *self, PyObject *args, PyObject *kwds) {
    // pass IC and HP into this
    PyObject *ic_ptr = NULL;
    PyObject *hp_ptr = NULL;
    if (PyArg_UnpackTuple(args, "args", 1, 2, &ic_ptr, &hp_ptr)) {
        ((FotPlanner *)self)->fot = new AnytimeFrenetOptimalTrajectory(
            ((FotIC *)ic_ptr)->ic, ((FotHP *)hp_ptr)->hp);
        printf("fot_planner: Anytime Fot Planner Initiated\n");
        return 0;
    } else {
        printf("Error: Anytime Fot Planner cannot be Initiated\n");
        return -1;
    }
}

/**
 * Deallocate FotPlanner and Planner Instance
 */
void FotPlanner_dealloc(FotPlanner *self) {
    printf("fot_planner: Planner dealloced\n");
    delete self->fot;
    Py_TYPE(self)->tp_free(self);
}

/**
 * Call Anytime Planner to initiate worker threads
 * and start planning based on ic and hp
 */
static PyObject *method_async_plan(PyObject *self, PyObject *args) {
    ((FotPlanner *)self)->fot->asyncPlan();
    printf("fot_planner: Anytime Fot Planner Start Planning Asynchronously\n");
    return Py_None;
}

/**
 * Get current best path from anytime planner,
 * Require a FotRV as argument, to store the current best path in it
 * Each query should require a different FotRV, so that different paths are
 * stored in separate objects
 */
static PyObject *method_get_path(PyObject *self, PyObject *args) {
    FrenetPath *best_frenet_path = ((FotPlanner *)self)->fot->getBestPath();

    PyObject *rv_ptr = NULL;
    FrenetReturnValues *fot_rv = NULL;

    if (PyArg_UnpackTuple(args, "args", 1, 1, &rv_ptr)) {
        fot_rv = ((FotRV *)rv_ptr)->rv;
    } else {
        printf("Require FotRV Object for Argument\n");
        return Py_None;
    }

    if (best_frenet_path && !best_frenet_path->x.empty()) {
        int last = 0;
        for (size_t i = 0; i < best_frenet_path->x.size(); i++) {
            fot_rv->x_path[i] = best_frenet_path->x[i];
            fot_rv->y_path[i] = best_frenet_path->y[i];
            fot_rv->speeds[i] = best_frenet_path->s_d[i];
            fot_rv->ix[i] = best_frenet_path->ix[i];
            fot_rv->iy[i] = best_frenet_path->iy[i];
            fot_rv->iyaw[i] = best_frenet_path->iyaw[i];
            fot_rv->d[i] = best_frenet_path->d[i];
            fot_rv->s[i] = best_frenet_path->s[i];
            fot_rv->speeds_x[i] =
                cos(best_frenet_path->yaw[i]) * fot_rv->speeds[i];
            fot_rv->speeds_y[i] =
                sin(best_frenet_path->yaw[i]) * fot_rv->speeds[i];
            last += 1;
        }

        // indicate last point in the path
        fot_rv->success = 1;
        fot_rv->x_path[last] = NAN;
        fot_rv->y_path[last] = NAN;
        fot_rv->speeds[last] = NAN;
        fot_rv->ix[last] = NAN;
        fot_rv->iy[last] = NAN;
        fot_rv->iyaw[last] = NAN;
        fot_rv->d[last] = NAN;
        fot_rv->s[last] = NAN;
        fot_rv->speeds_x[last] = NAN;
        fot_rv->speeds_y[last] = NAN;

        // store info for debug
        fot_rv->params[0] = best_frenet_path->s[1];
        fot_rv->params[1] = best_frenet_path->s_d[1];
        fot_rv->params[2] = best_frenet_path->d[1];
        fot_rv->params[3] = best_frenet_path->d_d[1];
        fot_rv->params[4] = best_frenet_path->d_dd[1];

        // store costs for logging
        fot_rv->costs[0] = best_frenet_path->c_lateral_deviation;
        fot_rv->costs[1] = best_frenet_path->c_lateral_velocity;
        fot_rv->costs[2] = best_frenet_path->c_lateral_acceleration;
        fot_rv->costs[3] = best_frenet_path->c_lateral_jerk;
        fot_rv->costs[4] = best_frenet_path->c_lateral;
        fot_rv->costs[5] = best_frenet_path->c_longitudinal_acceleration;
        fot_rv->costs[6] = best_frenet_path->c_longitudinal_jerk;
        fot_rv->costs[7] = best_frenet_path->c_time_taken;
        fot_rv->costs[8] = best_frenet_path->c_end_speed_deviation;
        fot_rv->costs[9] = best_frenet_path->c_longitudinal;
        fot_rv->costs[10] = best_frenet_path->c_inv_dist_to_obstacles;
        fot_rv->costs[11] = best_frenet_path->cf;
        return Py_None;
    } else {
        printf("No valid path available\n");
        return Py_None;
    }
}

/**
 * Stop Planning, calls stopPlanning from c planner
 */
static PyObject *method_stop_plan(PyObject *self, PyObject *args) {
    ((FotPlanner *)self)->fot->stopPlanning();
    printf("fot_planner: Planner Stopped Planning\n");
    return Py_None;
}

/**
 * Specify Methods for FotPlanner
 */
PyMethodDef FotPlanner_methods[] = {
    {"async_plan", method_async_plan, METH_VARARGS, "Start Async Planning"},
    {"get_path", method_get_path, METH_VARARGS, "Get Best Path"},
    {"stop_plan", method_stop_plan, METH_VARARGS,
     "Stop Planning, Shut Down all Workers"},
    {NULL, NULL, 0, NULL},
};

/**
 * Define FotPlanner as a PyObject
 * Link constructor and destructor methods
 */
PyTypeObject FotPlannerType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "fot_planner.FotPlanner",
    .tp_basicsize = sizeof(FotPlanner),
    .tp_dealloc = (destructor)FotPlanner_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc = "fot_planner.FotPlanner objects",
    .tp_methods = FotPlanner_methods,
    .tp_init = (initproc)FotPlanner_init,
    .tp_new = FotPlanner_new};

/**
 * Linking fot_planner to methods
 */
static struct PyModuleDef fot_planner_module = {
    PyModuleDef_HEAD_INIT, "fot_planner", "Anytime Fot Planner", -1,
    FotPlanner_methods};

/* Initialize the fot_planner module
 * TO-DO: Not sure if we need to incorporate Reference Count
 */
PyMODINIT_FUNC PyInit_fot_planner(void) {
    Py_Initialize();
    PyObject *m;
    if (PyType_Ready(&FotPlannerType) < 0)
        return NULL;

    m = PyModule_Create(&fot_planner_module);
    if (m == NULL)
        return NULL;

    PyModule_AddObject(m, "FotPlanner", (PyObject *)&FotPlannerType);
    printf("fot_planner Module imported!\n");
    fflush(stdout);
    return m;
}