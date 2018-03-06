#include "kuka_rsi3/TrajectoryGenerator.hpp"
#include <iostream>

#define TIME_STEP 0.05
int main(int argc, char ** argv)
{
    /** Settings **/
    /// Accelerations
    double a[DOF] = {3, 3, 3, 3, 3, 3};
    JointVal accel(a);

    /// Velocities
    double v[DOF] = {5, 5, 5, 5, 5, 5};
    JointVal vel(v);

    /// Initial generilized coorinates
    double ang1[DOF] = {30, -45, 0, 0, 9.2, 0};
    JointVal q1(ang1);

    /// Initial generilized coorinates
    double ang2[DOF] = {0, 90, -90, 0, 0, 45};
    JointVal q2(ang2);


    TrajectoryGenerator trajGen(accel, vel, TIME_STEP);
    if (trajGen.generateTrajectory(q1, q2)) {
        trajGen.writeToFile("Test.dat");
    }

    return 0;
}
