#include "AnytimeFrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"

#include <iostream>
#include <assert.h>
#include <unistd.h>

using namespace std;

int main() {

    double wx [25] = {132.67, 128.67, 124.67, 120.67, 116.67, 112.67, 108.67,
                   104.67, 101.43,  97.77,  94.84,  92.89,  92.4 ,  92.4 ,
                   92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.4 ,  92.39,  92.39,
                   92.39,  92.39,  92.39,  92.39};
    double wy [25] = {195.14, 195.14, 195.14, 195.14, 195.14, 195.14, 195.14,
                   195.14, 195.14, 195.03, 193.88, 191.75, 188.72, 185.32,
                   181.32, 177.32, 173.32, 169.32, 165.32, 161.32, 157.32,
                   153.32, 149.32, 145.32, 141.84};
    double o_llx[1] = {92.89};
    double o_lly[1] = {191.75};
    double o_urx[1] = {92.89};
    double o_ury[1] = {191.75};

    // set up experiment
    FrenetInitialConditions fot_ic = {
        34.6,
        7.10964962,
        -1.35277168,
        -1.86,
        0.0,
        10,
        wx,
        wy,
        25,
        o_llx,
        o_lly,
        o_urx,
        o_ury,
        1
    };
    FrenetHyperparameters fot_hp = {
        25.0,
        15.0,
        15.0,
        5.0,
        5.0,
        0.5,
        0.2,
        5.0,
        2.0,
        0.5,
        2.0,
        0.1,
        1.0,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        1.0,
        1.0,
        2 // num thread
    };

    // run experiment
    AnytimeFrenetOptimalTrajectory fot = AnytimeFrenetOptimalTrajectory(&fot_ic, &fot_hp);

    fot.asyncPlan(); // start planning

    const int NUM_ITER = 50;
    const int TEST_INTERVAL_TIME = 2; // in microseconds

    double prev_final_cf = INFINITY;
    double curr_cf;

    for (int i = 0; i < NUM_ITER; i ++) {
        FrenetPath* curr_frenet_path = fot.getBestPath();
        if (curr_frenet_path) {
            curr_cf = curr_frenet_path->cf;
            cout << "Found Valid Path with Cost: " << curr_cf << "\n";
        } else {
            curr_cf = INFINITY;
            cout << "No Valid Path Found\n";
        }

        // anytime algo consistency test, cost should only reduce or stay unchanged
        // assert(curr_cf > prev_final_cf);
        if (prev_final_cf < curr_cf) {
            cout << "Not Consistent\n";
            return -1; 
        }

        prev_final_cf = curr_cf;
        usleep(TEST_INTERVAL_TIME);
    }

    cout << "All iterations are consistent\n";

    fot.stopPlanning();

    return 0;
}