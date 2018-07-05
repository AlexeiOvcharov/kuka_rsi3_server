#ifndef TRAJECTORY_GENERATOR
#define TRAJECTORY_GENERATOR

#include <boost/filesystem/fstream.hpp>

#include <kuka_rsi3/DataTypes.hpp>
#include <kuka_rsi3/Params.hpp>
#include <utils/PrettyPrint.hpp>

#include <vector>
#include <string>
#include <iostream>

#define DEBUG true
#define TITLE false

class VelFunc
{
    public:

        VelFunc(double maxAccel = 0, double maxVel = 0)
        : ddq_m(maxAccel), dq_m(maxVel), valid(false)
        {}

        void setParams(double maxAccel, double maxVel)
        {
            ddq_m = maxAccel; dq_m = maxVel;
        }

        bool setPose(double initialPos, double endPos, double minPoseDiff)
        {
            q_i = initialPos;
            q_e = endPos;

            q_diff = std::abs(q_e - q_i);
            min_q_diff = minPoseDiff;
            diraction = (q_e > q_i) - (q_e < q_i);
            t0 = 5*minPoseDiff;
            calculateParameters();

            if (!checkTime() || q_diff > MAX_VEL) {
                std::cout << "Error: t2 < t1" << std::endl;
                std::cout << "Try correct max vel: ";
                if(maxVelCorrection(q_diff))
                    std::cout << dq_m << std::endl;
                else {
                    std::cout << "Fail!" << std::endl;
                    return false;
                }
                calculateParameters();
                // return false;
            }
            if (!checkPathLength(q_diff)) {
                std::cout << "Error: Path length is too small" << std::endl;
                if (DEBUG) std::cout << "q_diff: " << q_diff << "\t min_q_diff: " << min_q_diff << std::endl;
                return false;
            }
            return true;
        }

        double getVal(double x)
        {
            if (x >= t0 && x < t1)
                return diraction*ddq_m*(x - t0);
            if (x >= t1 && x < t2)
                return diraction*dq_m;
            if (x >= t2 && x < t3)
                return diraction*(dq_m - ddq_m*(x - t2));
            return 0;
        }

        void getTimeSpan(double * timeSpan)
        {
            timeSpan[0] = 0;
            timeSpan[1] = t3 + 5*min_q_diff;
        }

        double getVel()
        {
            return dq_m;
        }

        double getAccel()
        {
            return ddq_m;
        }

        bool valid;
    private:

        bool calculateParameters()
        {
            // Forward time calculation
            t1 = dq_m/ddq_m + t0;
            t2 = q_diff/dq_m + t0;
            t3 = t2 + t1 - t0;

            if (DEBUG) std::cout << "t \t [t0, t1, t2, t3] \t\t (" << t0 << ", " << t1 << ", " << t2 << ", " << t3 << ")" << std::endl;
            // if (DEBUG) std::cout << "Params \t [vel, accel, q_i, q_e] \t (" << dq_m << ", " << ddq_m << ", " << q_i << ", " << q_e << ")" << std::endl;
            return true;
        }

        bool checkTime()
        {
            if (t1 > t2) return false;
            else return true;
        }

        bool checkPathLength(double pathLength)
        {
            if (pathLength <= min_q_diff) return false;
            else  return true;
        }

        bool maxVelCorrection(double q_diff)
        {
            dq_m = 0.9*sqrt(ddq_m*q_diff);
            if (dq_m > MAX_VEL) dq_m = MAX_VEL;
            return true;
        }

        double t0, t1, t2, t3;
        double ddq_m, dq_m, q_i, q_e, min_q_diff, q_diff, diraction, shiftTime;
};

class TrajectoryGenerator
{
    public:
        TrajectoryGenerator(JointVal acceleration, JointVal velocity, double timeStep)
        : accel(acceleration), vel(velocity), T(timeStep)
        {
            funcVector.resize(DOF);
            for (size_t i = 0; i < DOF; ++i)
                funcVector[i].setParams(accel(i), vel(i));
        }

        // Trjectory generate ONLY for move PTP move
        bool generateTrajectory(JointVal startAng, JointVal endAng) {

            qTraj.resize(0);
            time.resize(0);

            JointVal dq = endAng - startAng;
            JointVal currAng;
            double timeSpan[] = {0, 0};
            double currTimeSpan[] = {0, 0};
            double maxTime = 0;

            for (size_t i = 0; i < DOF; ++i) {
                // If angle diff is small then the angle is constant (not valid)
                funcVector[i].valid = false;
                currAng(i) = startAng(i);
                if (MIN_TRAJ > abs(dq(i))) {
                    // if (DEBUG) std::cout << "LOW A" << (i + 1) << std::endl;
                    continue;
                }

                funcVector[i].valid = true;
                funcVector[i].setPose(startAng(i), endAng(i), MIN_TRAJ);
                funcVector[i].getTimeSpan(currTimeSpan);
                if (maxTime < currTimeSpan[1] - currTimeSpan[0]) {
                    maxTime = currTimeSpan[1] - currTimeSpan[0];
                    timeSpan[0] = currTimeSpan[0];
                    timeSpan[1] = currTimeSpan[1];
                }

                // Fill trajectory Information
                trajInfo.vel(i) = funcVector[i].getVel();
                trajInfo.accel(i) = funcVector[i].getAccel();
            }
            timeSpan[1] += MIN_TRAJ*5;

            for (double t = timeSpan[0]; t <= timeSpan[1]; t += T) {
                for (size_t i = 0; i < DOF; ++i) {
                    if (!funcVector[i].valid) {
                        continue;
                    }
                    currAng(i) += T/2 * (funcVector[i].getVal(t + T) + funcVector[i].getVal(t));
                }
                qTraj.push_back(currAng);
                time.push_back(t);
            }

            // Fill trajectory Information
            trajInfo.q_i = startAng;
            trajInfo.q_e = endAng;

            if (DEBUG)
                pp.printTrajInfo(trajInfo);

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

        TrajectoryInformation trajInfo;
        std::vector<VelFunc> funcVector;

        PrettyPrint pp;
};

#endif