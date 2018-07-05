#ifndef DATA_TYPES
#define DATA_TYPES

#include <matrix/math.hpp>
#include <kuka_rsi3/Params.hpp>

typedef matrix::Vector<double, DOF> JointVal;

struct TrajectoryInformation
{
    std::vector<double> time;
    JointVal accel, vel, q_i, q_e;
    double T;
};

#endif