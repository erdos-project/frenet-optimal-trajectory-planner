#ifndef FRENET_OPTIMAL_TRAJECTORY_ANYTIME_FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_ANYTIME_FRENET_OPTIMAL_TRAJECTORY_H

#include "CubicSpline2D.h"
#include "FrenetPath.h"
#include "Obstacle.h"
#include "py_cpp_struct.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <thread>
#include <vector>

using namespace std;
using namespace Eigen;

class AnytimeFrenetOptimalTrajectory {
  public:
    AnytimeFrenetOptimalTrajectory(FrenetInitialConditions *fot_ic_,
                                   FrenetHyperparameters *fot_hp_);
    ~AnytimeFrenetOptimalTrajectory();

    void asyncPlan();
    void stopPlanning();

    FrenetPath *getBestPath();
    void setObstacles();
    void addObstacle(Vector2f first_point, Vector2f second_point);

  private:
    FrenetInitialConditions *fot_ic;
    FrenetHyperparameters *fot_hp;
    mutex *mu;
    FrenetPath *best_frenet_path;
    vector<thread *> threads;
    CubicSpline2D *csp;
    vector<Obstacle *> obstacles;
    vector<double> x, y;
    vector<FrenetPath *> frenet_paths; // TO-DO: Considering using a heap
    bool run_workers;
    void
    calc_frenet_paths(int start_di_index,
                      int end_di_index); // using threaded implementation only
};

#endif // FRENET_OPTIMAL_TRAJECTORY_ANYTIME_FRENET_OPTIMAL_TRAJECTORY_H