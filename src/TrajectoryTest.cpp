#include "kuka_rsi3/TrajectoryGenerator.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
	double a = 3;
	double v = 5;
	double q1 = 0;
	double q2 = 90;

	TrajectoryGenerator trajGen(a, v);
	if (trajGen.generateTrajectory(q1, q2)) {
	    trajGen.writeToFile("Test.dat");
    }
	
	return 0;
}
