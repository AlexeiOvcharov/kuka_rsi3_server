#ifndef TRAJECTORY_GENERATOR
#define TRAJECTORY_GENERATOR

#include <boost/filesystem/fstream.hpp>

#include <vector>
#include <string>
#include <iostream>

#define DEBUG true
#define TITLE false
#define MIN_TRAJ 0.1
#define ACCEL_CORRECTION true
#define MAX_ACCEL 5
#define DOF 6

class TrajectoryGenerator 
{
    public:
        TrajectoryGenerator(double acceleration, double velocity, double timeStep)
        : accel(acceleration), vel(velocity), T(timeStep)
        {}

        bool generateTrajectory(double startAng, double endAng) {

            qTraj.resize(0);
            time.resize(0);

            double step = MIN_TRAJ;     // degrees
            double dq = endAng - startAng;
            int movementDirection = (dq > 0) - (dq < 0);

            if (MIN_TRAJ > abs(dq)) {
                std::cout << "Angle is too small!" << std::endl;
                return false;
            }

            double trajectoryTime[4];
            trajectoryTime[0] = 0;
            trajectoryTime[1] = vel/accel;
            trajectoryTime[2] = abs(dq)/vel;
            trajectoryTime[3] = trajectoryTime[1] + trajectoryTime[2];

            
            if (trajectoryTime[1] > trajectoryTime[2]) {
                std::cout << "Trajectory time for acceleration is too hight!" << std::endl;
                if (ACCEL_CORRECTION) {
                    std::cout << "Acceleration correction." << std::endl;
                    accel = vel/trajectoryTime[2];
                    trajectoryTime[1] = vel/accel;
                    trajectoryTime[3] = trajectoryTime[1] + trajectoryTime[2];
                } else return false;
            }
            if (DEBUG) std::cout << "[PARAMETERS] " << trajectoryTime[1] << ", " << trajectoryTime[2] << ", " << trajectoryTime[3] << std::endl;
            if (accel > MAX_ACCEL) {
                std::cout << "Acceleration is too hight!" << std::endl;
                return false;
            }

            double currTime = 0;
            double currAng = 0;
            for(; currTime <= trajectoryTime[1]; currTime += T) {
                currAng = movementDirection*accel*currTime*currTime/2 + startAng;
                qTraj.push_back(round(currAng*1000)/1000);
                time.push_back(currTime);
            }
            for(; currTime <= trajectoryTime[2]; currTime += T) {
                currAng = movementDirection*(vel*currTime - accel*trajectoryTime[1]*trajectoryTime[1]/2.0) + startAng;
                qTraj.push_back(round(currAng*1000)/1000);
                time.push_back(currTime);
            }
            for(; currTime <= trajectoryTime[3]; currTime += T) {
                currAng = movementDirection*(-accel*currTime*currTime/2.0
                 + accel*(trajectoryTime[1] + trajectoryTime[2])*currTime
                 - accel*(trajectoryTime[1]*trajectoryTime[1] + trajectoryTime[2]*trajectoryTime[2])/2.0) + startAng;
                qTraj.push_back(round(currAng*1000)/1000);
                time.push_back(currTime);
            }
            qTraj.push_back(currAng);
            time.push_back(currTime);

            return true;
        }

        void writeToFile(std::string fileName)
        {
            boost::filesystem::path p{fileName};
            boost::filesystem::ofstream ofs{p};
            if (TITLE) ofs << "q" << "\t" << "t" << "\n";
            for (size_t i = 0; i < time.size(); ++i) {
                ofs << qTraj[i] << "\t" << time[i] << "\n";
            }
            ofs.close();
        }

        std::vector<double> qTraj;
        std::vector<double> time; 
    private:
        double accel;
        double vel;
        double T;           // Time step
};

#endif
