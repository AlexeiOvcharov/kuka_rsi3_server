#ifndef TRAJECTORY_GENERATOR
#define TRAJECTORY_GENERATOR

#include <boost/filesystem/fstream.hpp>

#include <matrix/math.hpp>

#include <vector>
#include <string>
#include <iostream>

#define DEBUG true
#define TITLE false
#define MIN_TRAJ 0.1
#define ACCEL_CORRECTION true
#define VEL_CORRECTION true
#define MAX_ACCEL 5
#define MAX_VEL 10
#define VEL_STEP 0.1
#define DOF 6

typedef matrix::Vector<double, DOF> JointVal;

class TrajectoryGenerator 
{
    public:
        TrajectoryGenerator(JointVal acceleration, JointVal velocity, double timeStep)
        : accel(acceleration), vel(velocity), T(timeStep)
        {}

        // Trjectory generate ONLY for move PTP move
        bool generateTrajectory(JointVal startAng, JointVal endAng) {

            JointVal dq = endAng - startAng;
            JointVal movementDirection;
            JointVal currAng;
            double maxTime = 0;
            size_t maxTimeJoint = 0;
            double trajectoryTime[DOF][4];
            size_t points = 0;

            for (size_t i = 0; i < DOF; ++i) {
                // If angle diff is small then the angle is constant
                if (MIN_TRAJ > abs(dq(i))) {
                    movementDirection(i) = 0;
                    currAng(i) = startAng(i);
                    if (DEBUG) std::cout << "LOW A" << (i + 1) << std::endl;
                    continue;
                }

                movementDirection(i) = (dq(i) > 0) - (dq(i) < 0);
                trajectoryTime[i][1] = 0;
                trajectoryTime[i][1] = vel(i)/accel(i);
                trajectoryTime[i][2] = abs(dq(i))/vel(i);
                trajectoryTime[i][3] = trajectoryTime[i][1] + trajectoryTime[i][2];
                if (DEBUG) std::cout << "Accel: " << accel(i) << "\t | \t" << "Vel: " << vel(i) << std::endl;
                if (DEBUG) std::cout << "[PARAMETERS] (Without correction) " << trajectoryTime[i][1] << ", " << trajectoryTime[i][2] << ", " << trajectoryTime[i][3] << std::endl;

                while (trajectoryTime[i][1] > trajectoryTime[i][2]) {
                    std::cout << "Trajectory time for acceleration is too hight!" << std::endl;
                    // Accel correction
                    if (ACCEL_CORRECTION) {
                        std::cout << "Acceleration correction." << std::endl;
                        accel(i) = vel(i)/trajectoryTime[i][2];
                        trajectoryTime[i][1] = vel(i)/accel(i);
                        trajectoryTime[i][3] = trajectoryTime[i][1] + trajectoryTime[i][2];
                        if (DEBUG) std::cout << "[PARAMETERS] " << trajectoryTime[i][1] << ", " << trajectoryTime[i][2] << ", " << trajectoryTime[i][3] << std::endl;
                    } else return false;

                    if (accel(i) > MAX_ACCEL) {
                        std::cout << "Acceleration is too hight!" << std::endl;
                        if (VEL_CORRECTION) {
                            std::cout << "Try velocity correction!" << std::endl;
                            vel(i) -= VEL_STEP;
                            trajectoryTime[i][1] = vel(i)/accel(i);
                            trajectoryTime[i][2] = abs(dq(i))/vel(i);
                        } else return false;
                    }
                }
                // while (accel(i) > MAX_ACCEL) {
                //     std::cout << "Acceleration is too hight!" << std::endl;
                //     if (VEL_CORRECTION) {
                //         std::cout << "Try velocity correction!" << std::endl;
                //         vel(i) -= VEL_STEP;
                //         trajectoryTime[i][1] = vel(i)/accel(i);
                //         trajectoryTime[i][2] = abs(dq(i))/vel(i);
                //     }
                //     return false;
                // }

                if (trajectoryTime[i][3] > maxTime) {
                    maxTime = trajectoryTime[i][3];
                    maxTimeJoint = i;
                }
            }

            points = round(maxTime/T) + 1;
            qTraj.resize(points);
            time.resize(points);
            size_t i = 0;
            size_t p = 0;
            if (DEBUG) std::cout << "Points number: " << points << std::endl;

            for (double currTime = 0; currTime <= maxTime; currTime += T, ++p) {
                i = 0;
                for (; i < DOF; ++i) {
                    if (movementDirection(i) == 0) continue;
                    // Acceleration = max
                    if (currTime <= trajectoryTime[i][1]) {
                        currAng(i) = movementDirection(i)*accel(i)*currTime*currTime/2 + startAng(i);
                        // if (DEBUG) std::cout << "Accel = max" << std::endl;
                        // if (DEBUG) std::cout << "Angle: " << currAng(i) << std::endl;
                    }
                    // Acceleration = const
                    if (currTime > trajectoryTime[i][1] && currTime <= trajectoryTime[i][2]) {
                        currAng(i) = movementDirection(i)*(vel(i)*currTime - accel(i)*trajectoryTime[i][1]*trajectoryTime[i][1]/2.0) + startAng(i);
                        // if (DEBUG) std::cout << "Accel = 0" << std::endl;
                        // if (DEBUG) std::cout << "Angle: " << currAng(i) << std::endl;
                    }
                    // Acceleration = -max
                    if (currTime > trajectoryTime[i][2] && currTime <= trajectoryTime[i][3]) {
                        currAng(i) = movementDirection(i)*(-accel(i)*currTime*currTime/2.0
                         + accel(i)*(trajectoryTime[i][1] + trajectoryTime[i][2])*currTime
                         - accel(i)*(trajectoryTime[i][1]*trajectoryTime[i][1] + trajectoryTime[i][2]*trajectoryTime[i][2])/2.0) + startAng(i);
                        // if (DEBUG) std::cout << "Accel = -max" << std::endl;
                        // if (DEBUG) std::cout << "Angle: " << currAng(i) << std::endl;
                    }
                }

                // if (DEBUG) std::cout << "p: " << p << std::endl;

                qTraj[p] = currAng;
                time[p] = currTime;
            }
            if (DEBUG) std::cout << "Result: (p: " << p << ", " << "time.size(): " << time.size() << ")" << std::endl;
            if (p > time.size()) {
                std::cout << "Too many trajectory points!" << std::endl;
                return false;
            }
            return true;
        }

        void writeToFile(std::string fileName)
        {
            boost::filesystem::path p{fileName};
            boost::filesystem::ofstream ofs{p};
            // if (TITLE) ofs << "q" << "\t" << "t" << "\n";
            for (size_t i = 0; i < time.size(); ++i) {
                for (size_t j = 0; j < DOF; ++j)
                    ofs << qTraj[i](j) << "\t";
                ofs << time[i] << "\n";
            }
            ofs.close();
        }

        std::vector<JointVal> qTraj;
        std::vector<double> time; 
    private:
        JointVal accel;
        JointVal vel;
        double T;           // Time step
};

#endif
